// @file watch_table.rs
// @brief 注册条目、全局注册表 + watch_scalar! / watch_struct! 注册宏

use core::cell::RefCell;
use heapless::String;
use critical_section::Mutex;

use crate::debugger::watch_value::{WatchValueKind, Access};

/// 注册条目
#[derive(Clone, Copy)]
pub struct WatchEntry {
    pub path:      &'static str,          // "arm.target_height"
    pub parent:    &'static str,          // "arm" (顶级为 "")
    pub type_name: &'static str,          // "f32"
    pub kind:      WatchValueKind,        // F32
    pub access:    Access,                // ReadWrite / ReadOnly

    /// type-erased 指针 → &'static RefCell<实际类型>
    pub ptr: *const (),

    /// 读函数: ptr → borrow() → 序列化为字符串 (失败返回 None)
    pub read_fn:  fn(*const ()) -> Option<String<32>>,

    /// 写函数: ptr + &str → borrow_mut() → 反序列化写入 (返回是否成功)
    pub write_fn: fn(*const (), raw: &str) -> bool,
}


unsafe impl Send for WatchEntry {}

// 全局注册表

const MAX_ENTRIES: usize = 64;

pub struct WatchTable {
    entries: [Option<WatchEntry>; MAX_ENTRIES],
    count:   usize,
}

impl WatchTable {
    pub const fn new() -> Self {
        const NONE: Option<WatchEntry> = None;
        Self { entries: [NONE; MAX_ENTRIES], count: 0 }
    }

    pub fn register(&mut self, entry: WatchEntry) -> bool {
        if self.count >= MAX_ENTRIES { return false; }
        self.entries[self.count] = Some(entry);
        self.count += 1;
        true
    }

    pub fn get(&self, idx: usize) -> Option<&WatchEntry> {
        self.entries.get(idx).and_then(|e| e.as_ref())
    }

    pub fn len(&self) -> usize {
        self.count
    }

    pub fn find_by_path(&self, path: &str) -> Option<&WatchEntry> {
        self.entries.iter()
            .filter_map(|e| e.as_ref())
            .find(|e| e.path == path)
    }
}

/// 全局单例 — 初始化时注册, 运行时仅 task 读取
pub static WATCH_TABLE: Mutex<RefCell<WatchTable>> = Mutex::new(RefCell::new(WatchTable::new()));

/// 向全局注册表添加条目  初始化阶段被宏使用
pub fn register_watch(entry: WatchEntry) {
    critical_section::with(|cs| {
        WATCH_TABLE.borrow(cs).borrow_mut().register(entry);
    });
}

/// 在临界区内读取注册表执行操作 供 watch_task 使用
pub fn with_table<R>(f: impl FnOnce(&WatchTable) -> R) -> R {
    critical_section::with(|cs| {
        let table = WATCH_TABLE.borrow(cs).borrow();
        f(&table)
    })
}



/// 注册标量变量的宏, 生成一个 WatchEntry 并注册到全局 WATCH_TABLE
/// 用法：watch_scalar!("m3508_1_rpm", &M3508_1_RPM, ReadWrite);
#[macro_export]
macro_rules! watch_scalar {
    ($path:literal, $cell_ref:expr, $access:ident) => {{
        // 利用泛型函数推断 T: 从 &'static RefCell<T> 自动推导
        fn infer<T: $crate::debugger::watch_value::WatchValue>(
            cell: &'static ::core::cell::RefCell<T>,
            path: &'static str,
            access: $crate::debugger::watch_value::Access,
        ) -> $crate::debugger::watch_table::WatchEntry 
        {
            use $crate::debugger::watch_table::WatchEntry;
            use $crate::debugger::watch_value::WatchValue;
            WatchEntry {
                path,
                parent:    "",
                type_name: T::watch_type_name(),
                kind:      T::watch_kind(),
                access,
                ptr:       cell as *const ::core::cell::RefCell<T> as *const (),
                read_fn:   |ptr| {
                    let cell = unsafe { &*(ptr as *const ::core::cell::RefCell<T>) };
                    Some(T::watch_read(&cell.borrow()))
                },

                write_fn:  |ptr, raw| {
                    let cell = unsafe { &*(ptr as *const ::core::cell::RefCell<T>) };
                    if let Some(v) = T::watch_write(raw) 
                    {
                        *cell.borrow_mut() = v;
                        true
                    } 
                    else { false }
                },
            }
        }
        let entry = infer($cell_ref, $path, $crate::debugger::watch_value::Access::$access);
        $crate::debugger::watch_table::register_watch(entry);
    }};
}


/// 注册结构体字段的宏, 生成多个 WatchEntry 并注册到全局 WATCH_TABLE
/// 用法: watch_struct!("arm", ArmState, &ARM_STATE, { 
///     target_height: f32 => ReadWrite, 
///     current_height: f32 => ReadOnly 
///     });         
/// 每个字段生成一对 read_fn / write_fn, ptr 全部指向同一个 RefCell<StructType>
#[macro_export]
macro_rules! watch_struct {
    (
        $parent:literal,
        $struct_ty:ty,
        $cell_ref:expr,
        { $($field:ident : $field_ty:ty => $field_access:ident),+ $(,)? }
    ) => {{
        use $crate::debugger::watch_table::WatchEntry;
        use $crate::debugger::watch_value::{WatchValue, Access};
        $(
            {
                let _entry = WatchEntry {
                    path:      ::core::concat!($parent, ".", ::core::stringify!($field)),
                    parent:    $parent,
                    type_name: <$field_ty as WatchValue>::watch_type_name(),
                    kind:      <$field_ty as WatchValue>::watch_kind(),
                    access:    Access::$field_access,
                    ptr:       $cell_ref as *const ::core::cell::RefCell<$struct_ty> as *const (),

                    read_fn:   |ptr| {
                        let cell = unsafe { &*(ptr as *const ::core::cell::RefCell<$struct_ty>) };
                        // 只读, 不 borrow_mut
                        Some(<$field_ty as WatchValue>::watch_read(&cell.borrow().$field))
                    },

                    write_fn:  |ptr, raw| {
                        let cell = unsafe { &*(ptr as *const ::core::cell::RefCell<$struct_ty>) };

                        if let Some(v) = <$field_ty as WatchValue>::watch_write(raw) 
                        {
                            cell.borrow_mut().$field = v;
                            true
                        } 
                        else { false }
                    },
                };
                $crate::debugger::watch_table::register_watch(_entry);
            }
        )+
    }};
}
