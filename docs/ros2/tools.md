# ROS 2 好用工具

## PlotJuggler

[PlotJuggler](https://plotjuggler.io/) 可以用 GUI 的方式顯示 topic 數值隨時間的變化，他是開源的工具，程式碼可以在 [GitHub](https://github.com/facontidavide/PlotJuggler) 找到。如果有比較不同數值(只能是數值)之間隨時間的變化趨勢，可以用這個工具來觀察。

* 使用方式

```bash
# 安裝
sudo apt install ros-$ROS_DISTRO-plotjuggler-ros
# 使用
ros2 run plotjuggler plotjuggler
```

在使用上基本上就是 UI 的拖拉而已，可以建議大家跑個 turtlesim 來觀察。不過有個小地方要特別注意，如果你想要在 X 軸和 Y 軸各放上不同 topic 數值，記得拖拉時按著右鍵，這樣才能在不同軸放不同數值。
