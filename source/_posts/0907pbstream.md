---
title: ProtoStreamReader-map_builder类-cartographer
date: 2021-09-07 15:59:22
tags: cartographer
---

之前我们大致讲解了Node类的功能和使用，在Node类里面，有大量的map_builder_bridge的使用，今天开始记录一下，对map_builder_bridge的使用。

# 两个坐标系：local map frame 与 global map frame

```
/**
 * note: local map frame 与 global map frame
 * carographer中存在两个地图坐标系, 分别为global map frame与local map frame
 * 
 * local map frame
 * 是表达local slam结果的坐标系, 是固定的坐标系, 不会被回环检测与位姿图优化所更改, 
 * 其每一帧位姿间的坐标变换不会改变
 * 
 * global map frame
 * 是表达被回环检测与位姿图优化所更改后的坐标系, 当有新的优化结果可用时, 此坐标系与任何其他坐标系之间的转换都会跳变.
 * 它的z轴指向上方, 即重力加速度矢量指向-z方向, 即由加速度计测得的重力分量沿+z方向.
 */
```

# 结构体LocalTrajectoryData

这里包含着一些从局部SLAM接受到的轨迹数据，以及一些frame的变换

## LocalSlamData

LocalSlamData中包含了local slam的一些数据, 包含当前时间, 当前估计的位姿, 以及累计的所有雷达数据

```c++
    struct LocalSlamData {
      ::cartographer::common::Time time;
      ::cartographer::transform::Rigid3d local_pose;
      ::cartographer::sensor::RangeData range_data_in_local;
    };
```

## local_slam_data

```c++
std::shared_ptr<const LocalSlamData> local_slam_data;
```

指向前端局部SLAM数据的指针

## local_to_map

```c++
cartographer::transform::Rigid3d local_to_map;
```

local frame 到 global frame间的坐标变换

## published_to_tracking

```c++
std::unique_ptr<cartographer::transform::Rigid3d> published_to_tracking;
```

published_frame 到 tracking_frame 间的坐标变换

# 加载pbstream文件内容

```c++
// 加载pbstream文件
void MapBuilderBridge::LoadState(const std::string& state_filename,
                                 bool load_frozen_state) {
  // Check if suffix of the state file is ".pbstream".
  const std::string suffix = ".pbstream";
  // 检查后缀是否是.pbstream
  CHECK_EQ(state_filename.substr(
               std::max<int>(state_filename.size() - suffix.size(), 0)),
           suffix)
      << "The file containing the state to be loaded must be a "
         ".pbstream file.";
  LOG(INFO) << "Loading saved state '" << state_filename << "'...";
  // 加载文件内容
  cartographer::io::ProtoStreamReader stream(state_filename);
  // 解析数据
  map_builder_->LoadState(&stream, load_frozen_state);
}
```

首先检查传进来的filename是否是以pbstream结尾，通过

```
state_filename.size() - suffix.size()
```

找到字符串前面一个字符的位置，比如filename="a.pbstream"，则`a.size() - suffix.size()=1`，那么substr就会取从位置1到最后面的所有字符串，则为`.pbstream`

## 文件加载类ProtoStreamReader

```c++
// A reader of the format produced by ProtoStreamWriter.
class ProtoStreamReader : public ProtoStreamReaderInterface {
 public:
  explicit ProtoStreamReader(const std::string& filename);
  ~ProtoStreamReader() = default;

  ProtoStreamReader(const ProtoStreamReader&) = delete;
  ProtoStreamReader& operator=(const ProtoStreamReader&) = delete;

  bool ReadProto(google::protobuf::Message* proto) override;
  bool eof() const override;

 private:
  bool Read(std::string* decompressed_data);

  std::ifstream in_;
};
```

此处被调用的是构造函数

```c++
  explicit ProtoStreamReader(const std::string& filename);
```

下面是构造函数的实现

```c++
// 读取pbstream文件, 并对前8个字节的数据进行校验
ProtoStreamReader::ProtoStreamReader(const std::string& filename)
    : in_(filename, std::ios::in | std::ios::binary) {
  uint64 magic;
  // 对前8个字节的数据进行校验
  if (!ReadSizeAsLittleEndian(&in_, &magic) || magic != kMagic) {
    in_.setstate(std::ios::failbit);
  }
  CHECK(in_.good()) << "Failed to open proto stream '" << filename << "'.";
}
```

in_ 即为std::ifstream，`in_(filename, std::ios::in | std::ios::binary)`表示要打开的文件名字以及打开的方式

| mode        | description                  |
| :---------- | ---------------------------- |
| ios::in     | 为输入(读)而打开文件         |
| ios::out    | 为输出(写)而打开文件         |
| ios::ate    | 初始位置：文件尾             |
| ios::app    | 所有输出附加在文件末尾       |
| ios::trunc  | 如果文件已存在则先删除该文件 |
| ios::binary | 二进制方式                   |

这些方式是能够进行组合使用的，以“或”运算（“|”）的方式：例如

```cpp
std::ios::in | std::ios::binary
```

对文件的前八个字节进行校验，注意，所有的pbstream文件最前面有一个八个字节的校验码，每一次读取的时候需要先校验一下这八个字节的校验码是否正确。

```	c++
  if (!ReadSizeAsLittleEndian(&in_, &magic) || magic != kMagic) {
    in_.setstate(std::ios::failbit);
  }
```



### 数据校验ReadSizeAsLittleEndian

注意，所有的pbstream文件有一个八个字节的校验码，每一次读取的时候需要先校验一下这八个字节的校验码是否正确

```c++
// 读取前8个字节的值, 进行累加
bool ReadSizeAsLittleEndian(std::istream* in, uint64* size) {
  *size = 0;
  for (int i = 0; i != 8; ++i) {
    *size >>= 8;
    *size += static_cast<uint64>(in->get()) << 56;
  }
  return !in->fail();
}
```

这里的参数列表in的类型为`std::istream`，不同于`std::ifstream`是因为继承关系

```
ios_base <- ios <- istream <- ifstream
```

get()方法可以获得在文件中的第一个字节的值。这里假设传入的in的前面几个字节所对应的值为

```
127 		10			24 			10   		1 			...
01111111	00001010	00011000	00001010	00000001	...
```



**流程：**

``` 
size: 0
size >>= 8 : 即size向右边移动八位
size: 0

static_cast<uint64>(in->get()) << 56;  即第一个数字向左边移动56位,即七个字节。下面的数字第第几个字节（每个字节八位）
127 0	0	0	0	0	0	0
0	1	2	3	4	5	6	7

size += static_cast<uint64>(in->get()) << 56; 依然是上面这个数字

size >>= 8 :即size向右边移动八位
0	127 0	0	0	0	0	0
0	1	2	3	4	5	6	7

static_cast<uint64>(in->get()) << 56;  即第一个数字向左边移动56位,即七个字节。下面的数字第第几个字节（每个字节八位）
10	0	0	0	0	0	0	0
0	1	2	3	4	5	6	7

size += static_cast<uint64>(in->get()) << 56; 依然是上面这个数字
10	127 0	0	0	0	0	0
0	1	2	3	4	5	6	7
```

最终经过八次累加， 会得到一个和kmagic，即`const uint64 kMagic = 0x7b1d1f7b5bf501db;`一样的值。

