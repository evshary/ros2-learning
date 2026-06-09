---
title: Rust FFI
description: Rust 如何和 C 語言互通
keywords:
  - Rust
---

所謂 FFI 是 Foreign Function Interface，這是 Rust 用來和其他程式語言互通的界面。

一般來說在作業系統中，不同 library 要能互相呼叫，都需要遵守一定的規範，例如 data 在 memory 的 layout 或是 function call 的運作方式，這個我們稱之為 ABI (Application Binary Interface)。
當我們用 Rust 呼叫其他 library 或是相反時，就必須要清楚彼此的 ABI。

Rust 並沒有定義自己的 ABI，所以無法透過預先編譯好 dynamic library 來減少編譯時間。
而作為取捨，Rust 也因此可以進行內部優化，來讓性能提昇。

## Rust 呼叫 C function

假設我們有一個 C library 如下：

```c
#include <stdio.h>

typedef struct Person {
    const char *name;
    int age;
} Person;

void print_person(const Person *person) {
    printf("name = %s, age = %d\n", person->name, person->age);
}
```

在 Rust 端，可以用如下方式呼叫

```rust
// 引入 C 語言的 type
use std::ffi::CString;
use std::os::raw::{c_char, c_int};

// 使用 repr(C) 確保是 C 的 layout
#[repr(C)]
struct Person {
    // 注意需要對應到 C 的 type
    name: *const c_char,
    age: c_int,
}

// 要用 unsafe，因為這是外部 API
unsafe extern "C" {
    fn print_person(person: *const Person);
}

fn main() {
    // 用 CString 來使用 C 語言的字串
    let name = CString::new("Alice").unwrap();

    let person = Person {
        name: name.as_ptr(),
        age: 30,
    };

    // 呼叫外部 API 都需要用 unsafe
    unsafe {
        print_person(&person);
    }
}
```

如果每次都要自己處理 C function 的界面，那會太繁瑣，所以一般我們都會用 bindgen 來自動產生。
bindgen 可以把 C 的 `.h` 檔案自動轉換成 `.rs`（例如 `bindgen person.h -o bindings.rs`），裡面就包括我們上面看到的轉換。

我們也可以利用 Rust 編譯時的前置作業 `build.rs` 來產生 `bindings.rs`。
下面是完整範例：

```rust
use std::env;
use std::path::PathBuf;

fn main() {
    // C header path
    let header_path = "native/person.h";

    // 1. 告訴 Cargo：如果 header 改了，要重新跑 build.rs
    println!("cargo:rerun-if-changed={}", header_path);

    // 2. 告訴 Rust linker 去哪裡找 dynamic library
    println!("cargo:rustc-link-search=native");

    // 3. 告訴 Rust linker 連結 dynamic library
    //
    // Linux:   libperson.so
    // macOS:   libperson.dylib
    // Windows: person.dll + person.lib
    //
    // 這裡寫 person，不要寫 libperson.so
    println!("cargo:rustc-link-lib=dylib=person");

    // 4. 用 bindgen 從 C header 產生 Rust FFI bindings
    let bindings = bindgen::Builder::default()
        .header(header_path)
        .generate()
        .expect("Unable to generate bindings");

    // 5. 把產生的 bindings 寫到 OUT_DIR
    //
    // OUT_DIR 是 Cargo 提供的 build output 目錄
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings");
}
```

在 Rust 那邊要使用 include 來引入產生的 bindings.rs

```rust
// 引入 build.rs 產生的 bindings
include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

fn main() {
    //...
}
```

## C 呼叫 Rust function

如果我們要從 C 呼叫 Rust 的函式，先要有類似如下的定義

```rust
// C 語言的 type
use std::os::raw::{c_char, c_int};
use std::ffi::CStr;

// no_mangle 代表要 Rust 不要亂改 function symbol，這樣才能被找到
// extern "C" 代表要用 C 的 ABI
#[unsafe(no_mangle)]
pub extern "C" fn add(a: c_int, b: c_int) -> c_int {
    a + b
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn greet(name: *const c_char) {
    if name.is_null() {
        return;
    }

    let name = unsafe {
        CStr::from_ptr(name)
    };

    println!("Hello, {}", name.to_string_lossy());
}
```

在 `Cargo.toml` 需要產生 C 可連結的 library

```toml
[lib]
# 使用靜態 library，如 .a
crate-type = ["staticlib"]
# 如果是動態 library，要用下面的 type
#crate-type = ["cdylib"]
```

C 語言那邊則需要有對應的 header 檔

```h
#ifndef RUSTFFI_H
#define RUSTFFI_H

int add(int a, int b);
void greet(const char *name);

#endif
```

C 這邊可以用下面方式來連結 Rust library

```bash
# target/release 是搜尋路徑
# rustffi 是要連結的 library 名稱
gcc main.c -L target/release -lrustffi -o main
```
