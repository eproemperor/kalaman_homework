# OpenCV学习笔记 - RoboMaster装甲板识别相关知识点

## 1. 色彩空间转换

### `cv::cvtColor()`
```cpp
cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
```
**功能：** 图像色彩空间转换
- **参数1:** 输入图像 (frame)
- **参数2:** 输出图像 (hsv)
- **参数3:** 转换类型 (BGR → HSV)

**常用转换类型：**
- `COLOR_BGR2GRAY`:BGR转灰度
- `COLOR_BGR2HSV`:BGR转HSV
- `COLOR_BGR2RGB`:BGR转RGB
- `COLOR_HSV2BGR`:HSV转BGR

**HSV色彩空间优势:**
- H(色调):颜色类型(0-180)
- S(饱和度):颜色纯度(0-255)
- V(明度):亮度(0-255)
- 比RGB更易进行颜色分割

## 2. 颜色阈值分割

### `cv::inRange()`
```cpp
cv::inRange(hsv, YELLOW_LOWER, YELLOW_UPPER, mask);
```
**功能：** 提取指定范围内的像素
- **参数1:** 输入图像
- **参数2:** 下限阈值
- **参数3:** 上限阈值
- **参数4:** 输出二值掩码

**返回值：** 二值图像(255:在范围内,0:不在范围内)

### `cv::Scalar`
```cpp
const cv::Scalar RED_LOWER1 = cv::Scalar(0, 100, 100);
```
**功能：** 4元素向量容器,常用于表示颜色
- `Scalar(B, G, R)`:BGR颜色表示
- `Scalar(H, S, V)`:HSV阈值表示
- `Scalar(255, 255, 255)`：白色
- `Scalar(0, 0, 255)`:红色(BGR格式)

## 3. 形态学操作

### `cv::morphologyEx()`
```cpp
cv::morphologyEx(binary_img, result, cv::MORPH_OPEN, kernel);
cv::morphologyEx(result, result, cv::MORPH_CLOSE, kernel);
```
**功能：** 高级形态学变换
- **参数1:** 输入图像
- **参数2:** 输出图像
- **参数3:** 操作类型
- **参数4:** 结构元素（核）

**操作类型：**
- `MORPH_OPEN`:**开运算** = 腐蚀 → 膨胀
  - 作用：去除小噪点，分离相连物体
- `MORPH_CLOSE`:**闭运算** = 膨胀 → 腐蚀
  - 作用：填充小孔洞，连接临近物体
- `MORPH_GRADIENT`：膨胀图减腐蚀图
- `MORPH_TOPHAT`：原图减开运算图

### 结构元素创建
cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, 
                                           cv::Size(3, 3));
```
**形状类型：**
- `MORPH_RECT`：矩形核
- `MORPH_ELLIPSE`：椭圆形核
- `MORPH_CROSS`：十字形核

## 4. 基本数据结构

### `cv::Point2f`
```cpp
cv::Point2f center(x, y);
```
**功能：** 2D点坐标(float类型)
- `center.x`:x坐标
- `center.y`:y坐标
- `Point2i`：整数点
- `Point2d`：双精度点

### `cv::Rect`
```cpp
cv::Rect rect(x, y, width, height);
```
**功能：** 矩形区域定义
- **属性：** `x, y, width, height`
- **方法：**
  - `rect.contains(Point)`：判断点是否在矩形内
  - `rect.area()`：计算面积
  - `rect.tl()`：左上角点
  - `rect.br()`：右下角点

**创建方式：**
```cpp
cv::Rect rect1(10, 20, 100, 200);        // (x,y,w,h)
cv::Rect rect2 = rect1 + cv::Point(5,5);  // 平移
cv::Rect rect3 = rect1 & rect2;           // 交集
cv::Rect rect4 = rect1 | rect2;           // 并集
```

## 5. 霍夫圆检测

### `cv::HoughCircles()`
```cpp
cv::HoughCircles(cannon_area, circles, cv::HOUGH_GRADIENT, 
                 1, 50, 100, 30, 20, 60);
```
**功能：** 检测图像中的圆形
- **参数1:** 输入图像（灰度图）
- **参数2:** 输出向量（存储检测到的圆）
- **参数3:** 检测方法(目前仅HOUGH_GRADIENT)
- **参数4:** dp(累加器分辨率,1表示与原图相同)
- **参数5:** minDist(圆心最小距离)
- **参数6:** param1(Canny边缘检测高阈值)
- **参数7:** param2(圆心检测阈值,越小越敏感)
- **参数8:** minRadius(最小半径)
- **参数9:** maxRadius(最大半径)

**返回值：** `std::vector<cv::Vec3f>`
- `circle[0]`:圆心x坐标
- `circle[1]`:圆心y坐标
- `circle[2]`：半径

## 6. 轮廓处理

### `cv::findContours()`
```cpp
cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, 
                 cv::CHAIN_APPROX_SIMPLE);
```
**功能：** 查找图像轮廓
- **参数1:** 输入二值图像
- **参数2:** 输出轮廓点集
- **参数3:** 检索模式
- **参数4:** 近似方法

**检索模式：**
- `RETR_EXTERNAL`：只检测最外层轮廓
- `RETR_LIST`：检测所有轮廓，不建立层次
- `RETR_TREE`：检测所有轮廓，建立完整层次

**近似方法：**
- `CHAIN_APPROX_SIMPLE`：压缩水平、垂直、对角线段
- `CHAIN_APPROX_NONE`：存储所有轮廓点

### 轮廓属性计算
```cpp
double area = cv::contourArea(contour);           // 面积
cv::Rect bbox = cv::boundingRect(contour);        // 外接矩形
cv::Moments m = cv::moments(contour);              // 矩
cv::Point2f center(m.m10/m.m00, m.m01/m.m00);      // 质心
```

## 7. 位操作

### `cv::bitwise_and()` / `bitwise_or()` / `bitwise_not()`
```cpp
cv::bitwise_and(non_yellow, armor_region_mask, result);  // 与运算
cv::bitwise_not(yellow_mask, non_yellow);                 // 非运算
cv::bitwise_or(mask1, mask2, mask);                       // 或运算
```
**功能：** 图像位运算
- **作用：** 图像掩码操作、ROI提取
- **参数1:** 第一个图像
- **参数2:** 第二个图像
- **参数3:** 输出图像

## 8. 图像绘制

### `cv::circle()`
```cpp
cv::circle(frame, center, radius, color, thickness);
```
**功能：** 绘制圆形
- **thickness**:-1表示填充,正数表示线宽

### `cv::rectangle()`
```cpp
cv::rectangle(frame, rect, color, thickness);
cv::rectangle(frame, pt1, pt2, color, thickness);
```

### `cv::drawMarker()`
```cpp
cv::drawMarker(frame, center, color, marker_type, size, thickness);
```
**标记类型：**
- `MARKER_CROSS`：十字
- `MARKER_TILTED_CROSS`：斜十字
- `MARKER_STAR`：星形
- `MARKER_DIAMOND`：菱形
- `MARKER_SQUARE`：方形

### `cv::putText()`
```cpp
cv::putText(frame, text, org, font, scale, color, thickness);
```
**字体：** `FONT_HERSHEY_SIMPLEX`（常用）

## 9. 图像克隆

### `cv::Mat::clone()`
```cpp
current_frame = frame.clone();
```
**作用：** 深拷贝图像数据
- 避免修改原始图像
- 创建独立的内存副本

## 10. ROI提取

### 矩形区域提取
```cpp
cv::Mat roi = frame(rect);                    // ROI视图（浅拷贝）
cv::Mat roi_copy = frame(rect).clone();        // ROI深拷贝
```
- **浅拷贝：** 修改roi会影响原图
- **深拷贝：** 独立的数据副本

## 总结流程图

```
输入图像(BGR)
    ↓
cv::cvtColor → HSV图像
    ↓
cv::inRange → 颜色掩码
    ↓
cv::morphologyEx → 去噪掩码
    ↓
cv::findContours → 轮廓
    ↓
cv::moments → 质心坐标
    ↓
cv::circle/cv::putText → 可视化结果
```
