---
title: Rust trait 
description: Rust trait 概念
keywords:
  - Rust
---

trait 也就是屬性，在 Rust 中可以當成是用來修飾 struct 的形容詞，某種程度上可以當成是物件導向的 interface 的概念，代表這個 struct 實作了某些 API，其他人可以呼叫。

一個定義的 trait 通常下方會有多個 fn 需要由 struct 去實作。
我們用下面最簡單的範例來說明：

```rust
// 定義一個說話的 trait
trait Speak {
    fn say(&self) -> String;
}

// 隨便定義兩個 struct，分別是狗和人
struct Dog;
struct Human;

// 為 Dog 實作 Speak，代表狗會說話
impl Speak for Dog {
    fn say(&self) -> String {
        "汪汪".to_string()
    }
}

// 為 Human 實作 Speak，人也會說話
impl Speak for Human {
    fn say(&self) -> String {
        "你好".to_string()
    }
}

fn main() {
    let dog = Dog;
    let person = Human;

    // 我們可以直接讓這兩個 struct 說話
    println!("狗說：{}", dog.say());
    println!("人說：{}", person.say());
}
```

## Orphan Rule

當要為某個 type 實作 trait 時，要確保該 type 和 trait，至少有一個是在當前 crate 所定義，也就是不能為第三方的 type 定義第三方的 trait。

## impl Trait vs impl dyn Trait

[Rust：impl Trait vs impl dyn Trait](https://zhuanlan.zhihu.com/p/257090324)

[泛型与关联类型](https://rustcc.cn/article?id=fb4e1512-ca7a-4dfe-9c87-3c98e800ac23)

## Into & From

Into & From 兩者是兄弟，當定義 `From<T> for U` 也就隱含著 `Into<U> for T`，可以從 T 轉換成 U。
**但是反過來並不成立，實作 Into 不會有 From 的功能**，所以大部分我們都會選擇實作 From，除非在舊版要繞過 Orphan Rule，可參考[官方文件](https://doc.rust-lang.org/std/convert/trait.Into.html#implementing-into-for-conversions-to-external-types-in-old-versions-of-rust)。

* From

```rust
use std::convert::From;

struct Value {
    v: i32,
}

impl From<i32> for Value {
    fn from(i: i32) -> Self {
        Value { v: i }
    }
}

fn main() {
    let num1 = Value::from(30);
    let num2: Value = 40.into();
    println!("Result: {:} {:}", num1.v, num2.v);
}
```

* Into

```rust
use std::convert::Into;

struct Value {
    v: i32,
}

impl Into<Value> for i32 {
    fn into(self) -> Value {
        Value { v: self }
    }
}

fn main() {
    //let num1 = Value::from(30_i32);  // fail
    let num2: Value = 40_i32.into();
}
```

## TryInto & TryFrom

Into 和 From 並不允許轉換失敗，如果要支援轉換失敗，可用 TryInto 和 TryFrom。可參考 [TryFrom and TryInto](https://rustwiki.org/zh-CN/rust-by-example/conversion/try_from_try_into.html)

* TryFrom

```rust
use std::convert::TryFrom;

#[derive(Debug, PartialEq)]
struct PositiveNumber(i32);

impl TryFrom<i32> for PositiveNumber {
    type Error = ();

    fn try_from(v: i32) -> Result<Self, ()> {
        if v > 0 {
            Ok(PositiveNumber(v))
        } else {
            Err(())
        }
    }
}

fn main() {
    // TryFrom
    assert_eq!(PositiveNumber::try_from(8), Ok(PositiveNumber(8)));
    assert_eq!(PositiveNumber::try_from(-5), Err(()));

    // TryInto
    let result: Result<PositiveNumber, ()> = 8_i32.try_into();
    assert_eq!(result, Ok(PositiveNumber(8)));
    let result: Result<PositiveNumber, ()> = (-5_i32).try_into();
    assert_eq!(result, Err(()));
}
```

* TryInto

```rust
use std::convert::TryInto;

#[derive(Debug, PartialEq)]
struct PositiveNumber(i32);

impl TryInto<PositiveNumber> for i32 {
    type Error = ();

    fn try_into(self) -> Result<PositiveNumber, ()> {
        if self > 0 {
            Ok(PositiveNumber(self))
        } else {
            Err(())
        }
    }
}

fn main() {
    // TryFrom - compile fail
    //assert_eq!(PositiveNumber::try_from(8), Ok(PositiveNumber(8)));
    //assert_eq!(PositiveNumber::try_from(-5), Err(()));

    // TryInto
    let result: Result<PositiveNumber, ()> = 8_i32.try_into();
    assert_eq!(result, Ok(PositiveNumber(8)));
    let result: Result<PositiveNumber, ()> = (-5_i32).try_into();
    assert_eq!(result, Err(()));
}
```

## Rust 內建的 trait

主要分為三種：

* Auto Traits：能夠自動推導，如 `Send`、`Sync`、`Unpin`、`PanicUnwind`
* Derivable Traits：編譯器提供簡單的標籤 `#[derive(...)]` 提供快速實作，如 `Debug`、 `Clone`/`Copy`、`PartialEq`/`Eq`、`Default`
* Marker Traits：不需要做任何實作，單純只是編譯器用來貼標籤的，如 `Sized`、`Copy`

### auto trait

一般的 trait 都需要我們自己去 `impl Trait for MyStruct`，但是 Rust 有些內建 trait 會根據 struct 來自動推導。
只要該結構體內部所有成員都實作某個 `auto trait`，那這個結構體就自動實作該 trait，相反地如果有一個成員沒有實作，那這個結構體就不會實作。

#### Send & Sync

* Send：允許變數的 ownership 從某一個 thread 轉移到另一個 thread，兩個 thread 不會同時存取。

例如 `thread::spawn` 的 closure 使用 move 時，就要確保該變數有 Send。

```rust
use std::thread;

let v = vec![1, 2, 3];

// 因為 vec![] 有 Send，所以可以傳到別的 thread
thread::spawn(move || {
    println!("{:?}", v);
});
```

幾乎所有 Rust 變數都是 Send，但是有些例外，例如 `Rc<T>` 只能在 single thread 使用，需要 `Arc<T>` 才可以。

* Sync：Sync 允許多個 thread 在同個時間使用某個變數

官方的定義是如果 &T 是 Send，T 就是 Sync，也就是實現 Sync 的變數可以確保在多個 thread 中能擁有該變數的引用。

值得注意的是這邊實現 Sync 只能讀取該變數，如果是要能修改，那就需要使用 atomic 或是 `Mutex<T>`/`RwLock<T>` 才行。

* 兩者比較

會有 Send 和 Sync 最主要是在 compile-time 防止 data race。下面是兩者在各個 smart pointer 的比較。

| 型別 | Send | Sync |
| ------------ | ---- | ---- |
| `Rc<T>` | X | X |
| `Arc<T>` | O | O (但是 T 需要是 Send+Sync) |
| `RefCell<T>` | O | X (內部借用狀態不是 atomic，所以不能多人同時更新) |
| `Mutex<T>`/`RwLock<T>` | O | O |

特別注意一下像是 String、i32 都是同時有 Send 和 Sync，代表可以多個 thread 同時擁有。
然而在開 thread 的時候會需要使用 `Arc<T>` 是因為 Arc 可以確保內部 T 的 lifetime 是 `'static`。
傳遞的 String 和 i32 是 local 變數，有可能會活得比 thread 還短，所以才要用 Arc。
如果確定 thread 的 lifetime 會在當前區域結束，也可以用 `thread::scope`，這樣就不用 `Arc<T>`。

如果我們真的確認某個 struct 沒有 data race，那也可以自行用 unsafe trait 加上 Send 和 Sync。

```rust
unsafe impl Send for MyType {}
unsafe impl Sync for MyType {}
```

### Derivable Traits

#### Copy & Clone

實現 Copy 的類型的一定也是 Clone 的類型，Clone 是 Supertrait

Copy 的特性：

1. 只會完全複製 stack 的資料，不管 heap
2. 實作包在 Rust 內部，開發者無法修改
3. 觸發的時間點在幾個地方觸發：給值、參數傳入、回傳結果
4. 類型中的所有成員都必須是 Copy、並且沒有實現 Drop，該類型才能是 Copy
5. 絕大多數都是用 `#[derive(Copy, Clone)]` 來宣告，一般也會加上 Clone

Clone 的特性：

1. 複製 stack / heap 的資料都可以
2. 如果所有欄位都有實作 Clone，開發者可以直接用 `#[derive(Clone)]`
3. 如果沒有的話，那就必須要實作 `Clone::clone(&self)`
4. 觸發複製的時間點只有呼叫 `Clone::clone(&self)` 的時候
5. 類型沒有特別限制

這邊提供 impl Clone 的簡單範例：

```rust
struct Person {
    name: String,
}

impl Clone for Person {
    fn clone(&self) -> Self {
        Self {
            name: self.name.clone(),
        }
    }
}

fn main() {
    let p1 = Person {
        name: String::from("Alice"),
    };
    let p2 = p1.clone();

    println!("{}", p1.name);
    println!("{}", p2.name);
}
```

#### Debug

通常我們都會需要在 Debug 的時候印出 struct 的一些相關資訊，例如使用 `println!("{:?}", value);`。
這時怎麼顯示就必須要依靠 Debug 這個 trait。

我們可以用 `#[derive(Debug)]` 來使用預設的實作（但是 struct 內部必須都要有實作 Debug），也可以自行 impl。

根據 [Rust 官方建議](https://doc.rust-lang.org/std/fmt/#fmtdisplay-vs-fmtdebug)，所有公開的 struct 都應該要有實作 Debug，這是因為使用者預設就會認為他們可以直接印出相關細節，也更容易整合到不同開發工具之中。

當然，這也是會有副作用的，但是大體上都還在可以接受範圍

* 編譯時間和 code size 會稍微增加，不過跟帶來的好處想比，是可以被接受的
* 部份比較敏感的資訊會暴露出來，這部份可能會需要自行 impl Debug

我們可以用 `RUSTFLAGS="-W missing_debug_implementations" cargo check` 來確認當前專案到底有哪些公開的 struct 沒有 Debug 這個 trait。

簡單範例：

```rust
#[derive(Debug)]
struct Point {
    x: i32,
    y: i32,
}

fn main() {
    let p = Point { x: 3, y: 5 };

    println!("{:?}", p);
}
```

如果想要自訂輸出的內容，也可以自己實作 `Debug`：

```rust
use std::fmt;

struct User {
    name: String,
    age: u8,
}

impl fmt::Debug for User {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "User {{ name: {}, age: {} }}", self.name, self.age)
    }
}

fn main() {
    let user = User {
        name: String::from("Alice"),
        age: 18,
    };

    println!("{:?}", user);
}
```
