# 建图与定位

当前实现：运行官方 SLAM Toolbox 异步二维建图节点，消费 `/scan` 和
`odom -> base_footprint` TF，发布 `/map` 和 `map -> odom`。
参数基于官方 Humble 示例；不复制或重写扫描匹配、回环和优化算法。

后续已知地图定位的 AMCL / map_server 启动文件也放在本包。
建图和 AMCL 是互斥运行模式，不能同时发布 `map -> odom`。
