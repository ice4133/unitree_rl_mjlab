# URDF 转换为 MuJoCo XML

使用项目中的 MuJoCo 官方导入器，将机械提供的 URDF 和配套网格转换为 MJCF。

## 1. 放置文件

保留机械提供的完整目录结构。例如：

```text
src/assets/robots/my_robot_description/
├── package.xml          # 原包有这个文件时一起保留
├── urdf/
│   └── robot.urdf
└── meshes/
    ├── base.stl
    └── ...
```

网格可以沿用原文件名，无需改成 Go2W 的命名。

## 2. 转换

在项目根目录执行，把路径换成实际 URDF 路径：

```bash
conda activate unitree_rl_mjlab
python scripts/convert_urdf.py src/assets/robots/my_robot_description/urdf/robot.urdf
```

默认在 URDF 旁边生成 `robot.xml`，这就是 MJCF 文件。也可以指定输出位置：

```bash
python scripts/convert_urdf.py \
  src/assets/robots/my_robot_description/urdf/robot.urdf \
  --output src/assets/robots/my_robot_description/xmls/robot.xml
```

脚本会保留显示网格和固定连接的刚体，处理相对路径、绝对路径和常见的
`package://` 网格路径，并重新加载输出 XML 检查能否正常读取。
原始 URDF 和网格保持不变。输出 XML 继续引用原网格，移动模型时保留相对目录结构。

若提示找不到 `package://my_robot_description/...`，显式指定网格所属的包目录：

```bash
python scripts/convert_urdf.py \
  src/assets/robots/my_robot_description/urdf/robot.urdf \
  --package-root src/assets/robots/my_robot_description
```

输入应为已经展开的普通 URDF。输出文件已存在时，使用 `--output` 指定新文件名。

## 3. 打开模型

使用默认输出路径时：

```bash
python -m mujoco.viewer \
  --mjcf=src/assets/robots/my_robot_description/urdf/robot.xml
```

查看器中按空格可以暂停或继续。先暂停检查整机外观和装配位置。

看到 `MJCF saved and reloaded successfully` 表示转换及输出文件加载成功。
该命令不需要启动训练，也不依赖项目中的 Go2W 任务。
