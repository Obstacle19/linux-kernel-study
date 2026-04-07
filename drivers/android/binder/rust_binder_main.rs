// SPDX-License-Identifier: GPL-2.0

// Copyright (C) 2025 Google LLC.

//! Binder -- the Android IPC mechanism.

// 核心：用 Rust 实现一个 Binder 驱动，并把它挂到 Linux 内核的 VFS + binderfs 上

#![recursion_limit = "256"] // 把宏展开和类型推导等内部递归深度上限调到 256
#![allow( // 关闭下面这些 Clippy 警告
    clippy::as_underscore,
    clippy::ref_as_ptr,
    clippy::ptr_as_ptr,
    clippy::cast_lossless
)]

// 引入 kernel crate
use kernel::{
    // bindings（C 绑定层），rust/bindings/bindings_generated.rs
    // seq_file == sequential file，表示顺序文件
    bindings::{self, seq_file},
    // 文件系统，rust/kernel/fs.rs
    fs::File,
    // 内核链表，rust/kernel/list/
    list::{ListArc, ListArcSafe, ListLinksSelfPtr, TryNewListArc},
    // 通用导入，rust/kernel/prelude.rs
    prelude::*,
    seq_file::SeqFile,
    seq_print,
    sync::atomic::{ordering::Relaxed, Atomic},
    sync::poll::PollTable,
    // Arc == atomic reference counting，原子引用计数
    sync::Arc,
    task::Pid,
    transmute::AsBytes,
    types::ForeignOwnable,
    uaccess::UserSliceWriter,
};

// 引入 binder crate
// crate 在这里表示 drivers/android/binder
use crate::{context::Context, page_range::Shrinker, process::Process, thread::Thread};

// 引入 Rust 标准库自带的 core crate
use core::ptr::NonNull; // 引入非空指针类型，表示保证不为 NULL 的裸指针

// 使用 mod 声明把 binder 的主要子系统拉进来
mod allocation;
mod context;
mod deferred_close;
mod defs;
mod error;
mod node;
mod page_range;
mod process;
mod range_alloc;
mod stats;
mod thread;
mod trace;
mod transaction;

// 关闭这一段代码的所有编译警告
#[allow(warnings)] // generated bindgen code

// Rust 代码不会直接实现 binderfs，而是调用 C 里的函数
mod binderfs {
    // inode（index node），是文件的元数据入口
    // dentry（directory entry），目录项，是路径名字到 inode 的映射
    use kernel::bindings::{dentry, inode};

    // extern "C" 声明这个函数存在于别的地方，且需要用 C 的调用约定去调用它
    // 该声明仅提供符号签名，不包含实现
    // Rust 不知道实现在哪，也不负责找 .c 文件，函数解析完全依赖链接阶段：
    // 编译流程：
    //   Rust (.rs) → .o
    //   C    (.c ) → .o
    // 在 Makefile 里链接时，按符号名解析，若符号存在则链接成功；否则报 undefined reference
    extern "C" {
        // 初始化 binderfs 文件系统
        pub fn init_rust_binderfs() -> kernel::ffi::c_int; // root/rust/ffi.rs 里标明：c_int = i32;
    }
    /// 这里面rs和c类型转换时候会不会有 bug？

    // 为某个打开 binder 的进程创建按 pid 命名的调试文件：/dev/binderfs/binder_logs/proc/<pid>
    extern "C" {
        pub fn rust_binderfs_create_proc_file(
            nodp: *mut inode, // binderfs 的 inode
            pid: kernel::ffi::c_int, // 某个 binder 进程的 id
        ) -> *mut dentry;
    }

    // 删除 binderfs 中的调试文件
    extern "C" {
        pub fn rust_binderfs_remove_file(dentry: *mut dentry);
    }

    // 创建类型别名 rust_binder_context
    // rust_binder_context 用于在 C 和 Rust 之间传递 Binder 上下文对象
    // 对 C 来说，Rust 的 Context 是不透明类型，所以这里只能用 void* 表示
    // 所以 rust_binder_context 是由 Arc<Context> 转换来的不透明裸指针
    pub type rust_binder_context = *mut kernel::ffi::c_void; // root/rust/ffi.rs 里标明：pub use core::ffi::c_void;
    // 这里面 Arc<Context> 和 void* 的转换能不能 fuzz 出 bug？

    #[repr(C)] // 用 C 的内存布局来排列这个 struct
    #[derive(Copy, Clone)] // 表示这个 struct 可以按位拷贝
    pub struct binder_device {
        // 内核中主设备号用来标识设备驱动类型（如 binder 驱动），次设备号用来区分同一类型的不同设备实例（如 /dev/binder0、/dev/binder1 等）
        pub minor: kernel::ffi::c_int, // minor == minor number，表示次设备号
        pub ctx: rust_binder_context, // ctx == context，表示 binder 设备对应的上下文
    }
    impl Default for binder_device {
        fn default() -> Self { // default() 函数使用方式：let dev = binder_device::default();
            // 分配一块未初始化内存来存放 binder_device 结构体
            let mut s = ::core::mem::MaybeUninit::<Self>::uninit();
            // Rust 规定 手动操作内存是 unsafe 的，所以写在 unsafe 块里
            unsafe {
                ::core::ptr::write_bytes(s.as_mut_ptr(), 0, 1); // 将这块内存的每个字节都设置为 0，确保所有字段都被初始化为零值
                s.assume_init() // 将这块内存转换成 binder_device 结构体实例，并返回
            }
        }
    }
}

// 把 Rust Binder 注册成内核模块 rust_binder
module! {
    type: BinderModule, // 模块类型
    name: "rust_binder", // 模块名字
    authors: ["Wedson Almeida Filho", "Alice Ryhl"], // 模块作者
    description: "Android Binder", // 模块描述
    license: "GPL", // 模块许可证
}

use kernel::bindings::rust_binder_layout; // rust/bindings/bindings_generated.rs 里定义 rust_binder_layout 结构体
#[no_mangle] // 告诉 Rust 编译器不要对这个符号名做名字改写，以便 C 代码能通过符号名找到它
// 全局静态对象把三个模块导出的布局信息汇总成一个统一入口
static RUST_BINDER_LAYOUT: rust_binder_layout = rust_binder_layout {
    t: transaction::TRANSACTION_LAYOUT,
    p: process::PROCESS_LAYOUT,
    n: node::NODE_LAYOUT,
};

// 每调用一次 next_debug_id，就返回一个数字
// 这个数字来自一个全局计数器，第一次返回 0，第二次返回 1，第三次返回 2，依次递增
fn next_debug_id() -> usize {
    static NEXT_DEBUG_ID: Atomic<usize> = Atomic::new(0); // Atomic 可以保证并发加一不会数据竞争

    // fetch_add 的语义是：先把当前值读出来作为返回值，再做加一写回 NEXT_DEBUG_ID
    // Relaxed 表示这个操作不需要任何内存顺序保证，适合纯计数器这种不依赖其他内存操作的场景
    NEXT_DEBUG_ID.fetch_add(1, Relaxed)
}

/// Provides a single place to write Binder return values via the
/// supplied `UserSliceWriter`.
pub(crate) struct BinderReturnWriter<'a> {
    writer: UserSliceWriter,
    thread: &'a Thread,
}

impl<'a> BinderReturnWriter<'a> {
    fn new(writer: UserSliceWriter, thread: &'a Thread) -> Self {
        BinderReturnWriter { writer, thread }
    }

    /// Write a return code back to user space.
    /// Should be a `BR_` constant from [`defs`] e.g. [`defs::BR_TRANSACTION_COMPLETE`].
    fn write_code(&mut self, code: u32) -> Result {
        stats::GLOBAL_STATS.inc_br(code);
        self.thread.process.stats.inc_br(code);
        self.writer.write(&code)
    }

    /// Write something *other than* a return code to user space.
    fn write_payload<T: AsBytes>(&mut self, payload: &T) -> Result {
        self.writer.write(payload)
    }

    fn len(&self) -> usize {
        self.writer.len()
    }
}

/// Specifies how a type should be delivered to the read part of a BINDER_WRITE_READ ioctl.
///
/// When a value is pushed to the todo list for a process or thread, it is stored as a trait object
/// with the type `Arc<dyn DeliverToRead>`. Trait objects are a Rust feature that lets you
/// implement dynamic dispatch over many different types. This lets us store many different types
/// in the todo list.
trait DeliverToRead: ListArcSafe + Send + Sync {
    /// Performs work. Returns true if remaining work items in the queue should be processed
    /// immediately, or false if it should return to caller before processing additional work
    /// items.
    fn do_work(
        self: DArc<Self>,
        thread: &Thread,
        writer: &mut BinderReturnWriter<'_>,
    ) -> Result<bool>;

    /// Cancels the given work item. This is called instead of [`DeliverToRead::do_work`] when work
    /// won't be delivered.
    fn cancel(self: DArc<Self>);

    /// Should we use `wake_up_interruptible_sync` or `wake_up_interruptible` when scheduling this
    /// work item?
    ///
    /// Generally only set to true for non-oneway transactions.
    fn should_sync_wakeup(&self) -> bool;

    fn debug_print(&self, m: &SeqFile, prefix: &str, transaction_prefix: &str) -> Result<()>;
}

// Wrapper around a `DeliverToRead` with linked list links.
#[pin_data]
struct DTRWrap<T: ?Sized> {
    #[pin]
    links: ListLinksSelfPtr<DTRWrap<dyn DeliverToRead>>,
    #[pin]
    wrapped: T,
}
kernel::list::impl_list_arc_safe! {
    impl{T: ListArcSafe + ?Sized} ListArcSafe<0> for DTRWrap<T> {
        tracked_by wrapped: T;
    }
}
kernel::list::impl_list_item! {
    impl ListItem<0> for DTRWrap<dyn DeliverToRead> {
        using ListLinksSelfPtr { self.links };
    }
}

impl<T: ?Sized> core::ops::Deref for DTRWrap<T> {
    type Target = T;
    fn deref(&self) -> &T {
        &self.wrapped
    }
}

type DArc<T> = kernel::sync::Arc<DTRWrap<T>>;
type DLArc<T> = kernel::list::ListArc<DTRWrap<T>>;

impl<T: ListArcSafe> DTRWrap<T> {
    fn new(val: impl PinInit<T>) -> impl PinInit<Self> {
        pin_init!(Self {
            links <- ListLinksSelfPtr::new(),
            wrapped <- val,
        })
    }

    fn arc_try_new(val: T) -> Result<DLArc<T>, kernel::alloc::AllocError> {
        ListArc::pin_init(
            try_pin_init!(Self {
                links <- ListLinksSelfPtr::new(),
                wrapped: val,
            }),
            GFP_KERNEL,
        )
        .map_err(|_| kernel::alloc::AllocError)
    }

    fn arc_pin_init(init: impl PinInit<T>) -> Result<DLArc<T>, kernel::error::Error> {
        ListArc::pin_init(
            try_pin_init!(Self {
                links <- ListLinksSelfPtr::new(),
                wrapped <- init,
            }),
            GFP_KERNEL,
        )
    }
}

struct DeliverCode {
    code: u32,
    skip: Atomic<bool>,
}

kernel::list::impl_list_arc_safe! {
    impl ListArcSafe<0> for DeliverCode { untracked; }
}

impl DeliverCode {
    fn new(code: u32) -> Self {
        Self {
            code,
            skip: Atomic::new(false),
        }
    }

    /// Disable this DeliverCode and make it do nothing.
    ///
    /// This is used instead of removing it from the work list, since `LinkedList::remove` is
    /// unsafe, whereas this method is not.
    fn skip(&self) {
        self.skip.store(true, Relaxed);
    }
}

impl DeliverToRead for DeliverCode {
    fn do_work(
        self: DArc<Self>,
        _thread: &Thread,
        writer: &mut BinderReturnWriter<'_>,
    ) -> Result<bool> {
        if !self.skip.load(Relaxed) {
            writer.write_code(self.code)?;
        }
        Ok(true)
    }

    fn cancel(self: DArc<Self>) {}

    fn should_sync_wakeup(&self) -> bool {
        false
    }

    fn debug_print(&self, m: &SeqFile, prefix: &str, _tprefix: &str) -> Result<()> {
        seq_print!(m, "{}", prefix);
        if self.skip.load(Relaxed) {
            seq_print!(m, "(skipped) ");
        }
        if self.code == defs::BR_TRANSACTION_COMPLETE {
            seq_print!(m, "transaction complete\n");
        } else {
            seq_print!(m, "transaction error: {}\n", self.code);
        }
        Ok(())
    }
}

fn ptr_align(value: usize) -> Option<usize> {
    let size = core::mem::size_of::<usize>() - 1;
    Some(value.checked_add(size)? & !size)
}

// SAFETY: We call register in `init`.
static BINDER_SHRINKER: Shrinker = unsafe { Shrinker::new() };

struct BinderModule {}

impl kernel::Module for BinderModule {
    fn init(_module: &'static kernel::ThisModule) -> Result<Self> {
        // SAFETY: The module initializer never runs twice, so we only call this once.
        unsafe { crate::context::CONTEXTS.init() };

        pr_warn!("Loaded Rust Binder.");

        BINDER_SHRINKER.register(c"android-binder")?;

        // SAFETY: The module is being loaded, so we can initialize binderfs.
        unsafe { kernel::error::to_result(binderfs::init_rust_binderfs())? };

        Ok(Self {})
    }
}

/// Makes the inner type Sync.
#[repr(transparent)]
pub struct AssertSync<T>(T);
// SAFETY: Used only to insert `file_operations` into a global, which is safe.
unsafe impl<T> Sync for AssertSync<T> {}

/// File operations that rust_binderfs.c can use.
#[no_mangle]
#[used]
pub static rust_binder_fops: AssertSync<kernel::bindings::file_operations> = {
    // SAFETY: All zeroes is safe for the `file_operations` type.
    let zeroed_ops = unsafe { core::mem::MaybeUninit::zeroed().assume_init() };

    let ops = kernel::bindings::file_operations {
        owner: THIS_MODULE.as_ptr(),
        poll: Some(rust_binder_poll),
        unlocked_ioctl: Some(rust_binder_ioctl),
        compat_ioctl: bindings::compat_ptr_ioctl,
        mmap: Some(rust_binder_mmap),
        open: Some(rust_binder_open),
        release: Some(rust_binder_release),
        flush: Some(rust_binder_flush),
        ..zeroed_ops
    };
    AssertSync(ops)
};

/// # Safety
/// Only called by binderfs.
#[no_mangle]
unsafe extern "C" fn rust_binder_new_context(
    name: *const kernel::ffi::c_char,
) -> *mut kernel::ffi::c_void {
    // SAFETY: The caller will always provide a valid c string here.
    let name = unsafe { kernel::str::CStr::from_char_ptr(name) };
    match Context::new(name) {
        Ok(ctx) => Arc::into_foreign(ctx),
        Err(_err) => core::ptr::null_mut(),
    }
}

/// # Safety
/// Only called by binderfs.
#[no_mangle]
unsafe extern "C" fn rust_binder_remove_context(device: *mut kernel::ffi::c_void) {
    if !device.is_null() {
        // SAFETY: The caller ensures that the `device` pointer came from a previous call to
        // `rust_binder_new_device`.
        let ctx = unsafe { Arc::<Context>::from_foreign(device) };
        ctx.deregister();
        drop(ctx);
    }
}

/// # Safety
/// Only called by binderfs.

// extern "C" { fn foo(); } 表示函数在 C 实现
// extern "C" fn foo() {} 表示函数在 Rust 实现

// 这个函数是 binderfs 在打开 binder 设备文件时调用的回调函数
// 当用户态 open("/dev/binder") 时，内核（C）会调用这个 Rust 函数
// 它负责：创建 binder 进程上下文 + 绑定到 file
unsafe extern "C" fn rust_binder_open(
    inode: *mut bindings::inode, // binderfs 传入的 inode，表示被打开的设备文件
    file_ptr: *mut bindings::file, // binderfs 传入的 file，表示被打开的设备文件对应的内核 file 结构体
) -> kernel::ffi::c_int {
    // SAFETY: The `rust_binderfs.c` file ensures that `i_private` is set to a
    // `struct binder_device`.
    // i_private 是 binderfs 在注册设备时设置的私有字段，指向一个 binder_device 结构体，其中包含了 binder 设备对应的上下文信息
    let device = unsafe { (*inode).i_private } as *const binderfs::binder_device;

    // 断言 device 不为 NULL，否则说明 binderfs 没有正确设置 i_private 字段，无法继续处理这个 open 调用
    assert!(!device.is_null());

    // SAFETY: The `rust_binderfs.c` file ensures that `device->ctx` holds a binder context when
    // using the rust binder fops.
    // 从 device->ctx 取出 binder 全局上下文
    let ctx = unsafe { Arc::<Context>::borrow((*device).ctx) };

    // SAFETY: The caller provides a valid file pointer to a new `struct file`.
    // 
    let file = unsafe { File::from_raw_file(file_ptr) };
    // 创建一个新的 binder 进程，并把它绑定到这个文件上
    let process = match Process::open(ctx, file) {
        Ok(process) => process,
        Err(err) => return err.to_errno(),
    };

    // SAFETY: This is an `inode` for a newly created binder file.
    match unsafe { BinderfsProcFile::new(inode, process.task.pid()) } {
        Ok(Some(file)) => process.inner.lock().binderfs_file = Some(file),
        Ok(None) => { /* pid already exists */ }
        Err(err) => return err.to_errno(),
    }

    // SAFETY: This file is associated with Rust binder, so we own the `private_data` field.
    unsafe { (*file_ptr).private_data = process.into_foreign() };
    0
}

/// # Safety
/// Only called by binderfs.
unsafe extern "C" fn rust_binder_release(
    _inode: *mut bindings::inode,
    file: *mut bindings::file,
) -> kernel::ffi::c_int {
    // SAFETY: We previously set `private_data` in `rust_binder_open`.
    let process = unsafe { Arc::<Process>::from_foreign((*file).private_data) };
    // SAFETY: The caller ensures that the file is valid.
    let file = unsafe { File::from_raw_file(file) };
    Process::release(process, file);
    0
}

/// # Safety
/// Only called by binderfs.
unsafe extern "C" fn rust_binder_ioctl(
    file: *mut bindings::file,
    cmd: kernel::ffi::c_uint,
    arg: kernel::ffi::c_ulong,
) -> kernel::ffi::c_long {
    // SAFETY: We previously set `private_data` in `rust_binder_open`.
    let f = unsafe { Arc::<Process>::borrow((*file).private_data) };
    // SAFETY: The caller ensures that the file is valid.
    match Process::ioctl(f, unsafe { File::from_raw_file(file) }, cmd as _, arg as _) {
        Ok(()) => 0,
        Err(err) => err.to_errno() as isize,
    }
}

/// # Safety
/// Only called by binderfs.
unsafe extern "C" fn rust_binder_mmap(
    file: *mut bindings::file,
    vma: *mut bindings::vm_area_struct,
) -> kernel::ffi::c_int {
    // SAFETY: We previously set `private_data` in `rust_binder_open`.
    let f = unsafe { Arc::<Process>::borrow((*file).private_data) };
    // SAFETY: The caller ensures that the vma is valid.
    let area = unsafe { kernel::mm::virt::VmaNew::from_raw(vma) };
    // SAFETY: The caller ensures that the file is valid.
    match Process::mmap(f, unsafe { File::from_raw_file(file) }, area) {
        Ok(()) => 0,
        Err(err) => err.to_errno(),
    }
}

/// # Safety
/// Only called by binderfs.
unsafe extern "C" fn rust_binder_poll(
    file: *mut bindings::file,
    wait: *mut bindings::poll_table_struct,
) -> bindings::__poll_t {
    // SAFETY: We previously set `private_data` in `rust_binder_open`.
    let f = unsafe { Arc::<Process>::borrow((*file).private_data) };
    // SAFETY: The caller ensures that the file is valid.
    let fileref = unsafe { File::from_raw_file(file) };
    // SAFETY: The caller ensures that the `PollTable` is valid.
    match Process::poll(f, fileref, unsafe { PollTable::from_raw(wait) }) {
        Ok(v) => v,
        Err(_) => bindings::POLLERR,
    }
}

/// # Safety
/// Only called by binderfs.
unsafe extern "C" fn rust_binder_flush(
    file: *mut bindings::file,
    _id: bindings::fl_owner_t,
) -> kernel::ffi::c_int {
    // SAFETY: We previously set `private_data` in `rust_binder_open`.
    let f = unsafe { Arc::<Process>::borrow((*file).private_data) };
    match Process::flush(f) {
        Ok(()) => 0,
        Err(err) => err.to_errno(),
    }
}

/// # Safety
/// Only called by binderfs.
#[no_mangle]
unsafe extern "C" fn rust_binder_stats_show(
    ptr: *mut seq_file,
    _: *mut kernel::ffi::c_void,
) -> kernel::ffi::c_int {
    // SAFETY: The caller ensures that the pointer is valid and exclusive for the duration in which
    // this method is called.
    let m = unsafe { SeqFile::from_raw(ptr) };
    if let Err(err) = rust_binder_stats_show_impl(m) {
        seq_print!(m, "failed to generate state: {:?}\n", err);
    }
    0
}

/// # Safety
/// Only called by binderfs.
#[no_mangle]
unsafe extern "C" fn rust_binder_state_show(
    ptr: *mut seq_file,
    _: *mut kernel::ffi::c_void,
) -> kernel::ffi::c_int {
    // SAFETY: The caller ensures that the pointer is valid and exclusive for the duration in which
    // this method is called.
    let m = unsafe { SeqFile::from_raw(ptr) };
    if let Err(err) = rust_binder_state_show_impl(m) {
        seq_print!(m, "failed to generate state: {:?}\n", err);
    }
    0
}

/// # Safety
/// Only called by binderfs.
#[no_mangle]
unsafe extern "C" fn rust_binder_proc_show(
    ptr: *mut seq_file,
    _: *mut kernel::ffi::c_void,
) -> kernel::ffi::c_int {
    // SAFETY: Accessing the private field of `seq_file` is okay.
    let pid = (unsafe { (*ptr).private }) as usize as Pid;
    // SAFETY: The caller ensures that the pointer is valid and exclusive for the duration in which
    // this method is called.
    let m = unsafe { SeqFile::from_raw(ptr) };
    if let Err(err) = rust_binder_proc_show_impl(m, pid) {
        seq_print!(m, "failed to generate state: {:?}\n", err);
    }
    0
}

/// # Safety
/// Only called by binderfs.
#[no_mangle]
unsafe extern "C" fn rust_binder_transactions_show(
    ptr: *mut seq_file,
    _: *mut kernel::ffi::c_void,
) -> kernel::ffi::c_int {
    // SAFETY: The caller ensures that the pointer is valid and exclusive for the duration in which
    // this method is called.
    let m = unsafe { SeqFile::from_raw(ptr) };
    if let Err(err) = rust_binder_transactions_show_impl(m) {
        seq_print!(m, "failed to generate state: {:?}\n", err);
    }
    0
}

fn rust_binder_transactions_show_impl(m: &SeqFile) -> Result<()> {
    seq_print!(m, "binder transactions:\n");
    let contexts = context::get_all_contexts()?;
    for ctx in contexts {
        let procs = ctx.get_all_procs()?;
        for proc in procs {
            proc.debug_print(m, &ctx, false)?;
            seq_print!(m, "\n");
        }
    }
    Ok(())
}

fn rust_binder_stats_show_impl(m: &SeqFile) -> Result<()> {
    seq_print!(m, "binder stats:\n");
    stats::GLOBAL_STATS.debug_print("", m);
    let contexts = context::get_all_contexts()?;
    for ctx in contexts {
        let procs = ctx.get_all_procs()?;
        for proc in procs {
            proc.debug_print_stats(m, &ctx)?;
            seq_print!(m, "\n");
        }
    }
    Ok(())
}

fn rust_binder_state_show_impl(m: &SeqFile) -> Result<()> {
    seq_print!(m, "binder state:\n");
    let contexts = context::get_all_contexts()?;
    for ctx in contexts {
        let procs = ctx.get_all_procs()?;
        for proc in procs {
            proc.debug_print(m, &ctx, true)?;
            seq_print!(m, "\n");
        }
    }
    Ok(())
}

fn rust_binder_proc_show_impl(m: &SeqFile, pid: Pid) -> Result<()> {
    seq_print!(m, "binder proc state:\n");
    let contexts = context::get_all_contexts()?;
    for ctx in contexts {
        let procs = ctx.get_procs_with_pid(pid)?;
        for proc in procs {
            proc.debug_print(m, &ctx, true)?;
            seq_print!(m, "\n");
        }
    }
    Ok(())
}

struct BinderfsProcFile(NonNull<bindings::dentry>);

// SAFETY: Safe to drop any thread.
unsafe impl Send for BinderfsProcFile {}

impl BinderfsProcFile {
    /// # Safety
    ///
    /// Takes an inode from a newly created binder file.
    unsafe fn new(nodp: *mut bindings::inode, pid: i32) -> Result<Option<Self>> {
        // SAFETY: The caller passes an `inode` for a newly created binder file.
        let dentry = unsafe { binderfs::rust_binderfs_create_proc_file(nodp, pid) };
        match kernel::error::from_err_ptr(dentry) {
            Ok(dentry) => Ok(NonNull::new(dentry).map(Self)),
            Err(err) if err == EEXIST => Ok(None),
            Err(err) => Err(err),
        }
    }
}

impl Drop for BinderfsProcFile {
    fn drop(&mut self) {
        // SAFETY: This is a dentry from `rust_binderfs_remove_file` that has not been deleted yet.
        unsafe { binderfs::rust_binderfs_remove_file(self.0.as_ptr()) };
    }
}
