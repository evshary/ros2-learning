---
title: Rust iterator 介紹
description: 列出常用 Rust iterator 操作
keywords:
  - Rust
---

iterator 可以想成是一個個吐出元素的型態，例如：

```rust
// 我們有個 vector
let v = vec![1, 2, 3];
// 可以轉成 iterator
let iter = v.iter();
// 用 collect 轉回 vector，型態可以寫在左邊或右邊，下面兩者都可以接受
let result: Vec<_> = iter.collect();
let result = iter.collect::<Vec<_>>();
```

常用轉成 iterator 的方法有：

* `v.iter()`：產生 `&T`，不取得所有權
* `v.iter_mut()`：產生 `&mut T`，可修改元素
* `v.into_iter()`：產生 `T`，取得所有權

接下來介紹常用的 iterator 操作：

## map

每個元素進來，轉換成另外的東西出去

```rust
let nums = vec![1, 2, 3];
let doubled: Vec<_> = nums.iter()
    .map(|x| x * 2)
    .collect();
println!("{:?}", doubled); // [2, 4, 6]
```

## filter

留下符合條件的元素

```rust
let nums = vec![1, 2, 3, 4, 5];
// 這邊的 x 是 &&i32，因為 filter 是拿到 &Item
let even: Vec<_> = nums.iter()
    .filter(|x| **x % 2 == 0)
    .collect();
println!("{:?}", even); // [2, 4]
```

## filter_map

filter + map，如果是 Some(x)，留下 x，而 None 則是丟掉

```rust
let inputs = vec!["1", "abc", "3"];
// 只有 ok() 會把 Ok(i32) 轉成 Some(i32)
let nums: Vec<i32> = inputs.iter()
    .filter_map(|s| s.parse::<i32>().ok())
    .collect();
println!("{:?}", nums); // [1, 3]
```

## for_each

對每個元素做事，不產生新結果，但有時候用 for 更簡單

```rust
let nums = vec![1, 2, 3];
nums.iter().for_each(|x| {
    println!("{}", x);
});
```

## find

找到第一個符合的值，回傳 `Option<&T>`

```rust
let nums = vec![1, 2, 3, 4];
// 注意 x 是 &&i32
let first_even = nums.iter()
    .find(|x| **x % 2 == 0);
println!("{:?}", first_even); // Some(2)
```

## any, all

* any：是否有任一元素符合條件
* all：是否全部元素都符合條件

```rust
let nums = vec![1, 2, 3];
// 有一個是偶數
let has_even = nums.iter().any(|x| x % 2 == 0);
println!("{}", has_even); // true
// 全部都是偶數
let all_even = nums.iter().all(|x| x % 2 == 0);
println!("{}", all_even); // false
```

## count

算 iterator 有多少數量，常常跟 filter 搭配，記住這會消耗 iterator

```rust
let nums = vec![1, 2, 3, 4, 5];
// 偶數的數量為何
let even_count = nums.iter()
    .filter(|x| **x % 2 == 0)
    .count();
println!("{}", even_count); // 2
```

## flatten

把巢狀 iterator 攤平

```rust
// 雙層 Vector
let nested = vec![vec![1, 2], vec![3, 4]];
let flat: Vec<_> = nested.into_iter()
    .flatten()
    .collect();
println!("{:?}", flat); // [1, 2, 3, 4]
// 拿掉 Option
let values = vec![Some(1), None, Some(3)];
let result: Vec<_> = values.into_iter()
    .flatten()
    .collect();
println!("{:?}", result); // [1, 3]
```

## 取值：nth、take、skip

可以一個個從 iterator 取值，記住會消耗 iterator

```rust
let nums = vec![1, 2, 3, 4, 5, 6, 7, 8, 9];
let mut iter = nums.iter();
// 直接取第 2 個數值
println!("{:?}", iter.nth(1)); // Some(2)
println!("{:?}", iter.next()); // Some(3)
// 取下 3 個，要 clone 是因為會拿走 iterator ownership
let next_three: Vec<_> = iter.clone()
    .take(3)
    .collect();
println!("{:?}", next_three); // [4, 5, 6]
// 跳過 2 個，取剩下的，要 clone 是因為會拿走 iterator ownership
let after_two: Vec<_> = iter.clone()
    .skip(2)
    .collect();
println!("{:?}", after_two); // [6, 7, 8, 9]
```

## 合併：enumerate、zip、chain

* enumerate：會結合 index
* zip：結合兩個 iterator，以最短的為主
* chain：把兩個 iterator 串起來

```rust
// 結合 index
let words = vec!["a", "b", "c"];
for (idx, word) in words.iter().enumerate() {
    println!("{}: {}", idx, word);
}

// 結合兩個 iterator
let names = vec!["Alice", "Bob"];
let scores = vec![90, 80];
for (name, score) in names.iter().zip(scores.iter()) {
    println!("{}: {}", name, score);
}

// 串起兩個 iterator
let a = vec![1, 2];
let b = vec![3, 4];
let result: Vec<_> = a.iter()
    .chain(b.iter())
    .collect();
println!("{:?}", result); // [1, 2, 3, 4]
```

## String 相關：chars、split_whitespace、split、lines

* chars：把 String 切成一個個 character

```rust
let s = "12345";
// 是否全為數字
let ok = s.chars().all(|c| c.is_ascii_digit());
```

* split_whitespace：用 whitespace 分割 String

```rust
let line = "hello   rust world";
let words: Vec<_> = line.split_whitespace().collect();
println!("{:?}", words); // ["hello", "rust", "world"]
```

* split：用某個字元分割

```rust
let line = "apple,banana,orange";
let parts: Vec<_> = line.split(',').collect();
println!("{:?}", parts); // ["apple", "banana", "orange"]
```

* lines：用 \n 分割
    * 注意讀檔的 `reader.lines()` 是回傳 `io::Result<String>`，兩者不同

```rust
let text = "a\nb\nc";
for line in text.lines() {
    println!("{}", line);
}
```
