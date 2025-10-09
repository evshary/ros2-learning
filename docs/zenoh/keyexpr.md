---
title: Zenoh key
description: 介紹 Zenoh 的 key expression 相關規則
keywords:
  - Zenoh
---

Zenoh 對上層使用者來說就是提供 key/value 的配對。
使用者不需要知道對方位置，只需要對 key 做操作即可彼此通訊。
在其他 protocol 中，通常會稱之為 topic，但因為在 Zenoh 中還有其他的特性，所以我們稱之為 key。

一個典型的例子是 `organizationA/building8/room275/sensor3/temperature`，代表著機構 A 的第 8 棟建築下房間 275 的 3 號 sensor 的溫度。

我們可以看出 key 有兩個主要的特性：

1. 階層關係：Zenoh 可以利用 `/` 來把一個 key 分成多個 chunk，這樣在比較兩個 key 的時候可以有效降低計算量。
2. key expression：我們可以利用一些特殊字符來涵蓋多個 key，就如同 regular expression 一樣。

因此 key 中有些規則必須要遵守：

* 開頭結尾不能是 `/`，且在 key 內部也不能連續出現，例如 `//`
* key 必須是非空的 UTF-8 字元
* key 內部不能用到一些保留字元，如 `*`, `$`, `?`, `#`，這些會在 key expression 中使用到

## key expression

Key expression 最大的用途在於，我們可以用他來對多個 key 進行存取，他主要的設計來自於 [Key Expression Language](https://github.com/eclipse-zenoh/roadmap/blob/main/rfcs/ALL/Key%20Expressions.md)。

主要特性有：

* `*` 只能單獨出現在 chunk 中，代表著可以對應到任意的字元，例如 `organizationA/building8/room275/*/temperature` 就包含了 room275 底下任何 sensor 的 temperature。
* `$*` 一樣可以對應到任意字元，但是可以和其他字元一起出現在 chunk 之中，例如 `organizationA/building8/room275/sensor$*/temperature` 就是取得 sensor1, sensor2 等等感測器的溫度。
* `**` 可以對應到 0 到多個 chunk，例如 `organizationA/**/temperature` 可以取得所有機構 A 下的所有溫度資訊。

為了避免冗於設計，也就是多種 key expression 對應到相同意思，有些規則必須要遵守：

* 只允許一個 `**`，不能有 `**/**`
* 只允許 `*/**`，不能有 `**/*`
* 只允許單一 `$*`，不能有 `$*$*`
* 在一個 chunk 中，不能單獨存在 `$*`，只能用 `*` 替換

!!! warning
    少用 `$*`，因為效能較差，盡量多用 `*` 替換。例如 `robot/12` 和 `robot/18` 的設計就比 `robot12` 和 `robot12` 好，因為我們可以用 `robot/*` 來取得所有的 robot，而不是用效能較差的 `robot$*`

## Selector

[Selector](https://github.com/eclipse-zenoh/roadmap/blob/main/rfcs/ALL/Selectors/README.md)基本上就是 key expression 加上 parameters。
這個只能使用在 Query 上，主要目的是提供 Query 更多可以判斷的參數。
使用方式跟 HTTP URL 如何帶入 GET 參數很類似，在 key expression 後方加上 `?`，就可以放入多個 key/value 的配對了，中間一樣用 `;` 來做區隔。

舉例來說，下面的 Selector 可以區分為 Key Expression 和 parameters 兩個部份

```raw
path/**/something?arg1=val1;arg2=value%202
^               ^ ^                      ^
|Key Expression-| |----- parameters -----|
```

在程式內部可以得到

```raw
{
  key_expr: "path/**/something",
  parameters: {arg1: "val1", arg2: "value 2"}
}
```

## 參考連結

* [Zenoh Abstraction](https://zenoh.io/docs/manual/abstractions/)：這邊有提供完整的 key, key expression, selector 的說明
* [Zenoh RFC](https://github.com/eclipse-zenoh/roadmap/tree/main/rfcs/)：如果要有明確的規格，可以參考 RFC
