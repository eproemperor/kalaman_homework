# C++ STL 容器完全指南

## 1. 容器概述

STL（Standard Template Library）容器分为三类：
- **序列容器**：维护元素的线性顺序
- **关联容器**：基于键值自动排序
- **无序关联容器**：基于哈希表，不排序

| 类型 | 容器 | 特点 |
|------|------|------|
| 序列 | `vector` | 动态数组，随机访问快 |
| 序列 | `deque` | 双端队列，两端操作快 |
| 序列 | `list` | 双向链表，中间插入/删除快 |
| 序列 | `forward_list` | 单向链表（C++11） |
| 序列 | `array` | 固定大小数组（C++11） |
| 关联 | `set` / `multiset` | 有序集合，键唯一/可重复 |
| 关联 | `map` / `multimap` | 键值对，键唯一/可重复 |
| 无序 | `unordered_set/map` | 哈希实现，O(1)查找 |

## 2. vector（动态数组）

### 2.1 基本操作

```cpp
#include <vector>
using namespace std;

vector<int> v;                    // 空vector
vector<int> v2(5, 10);           // 5个10
vector<int> v3 = {1, 2, 3, 4};   // 初始化列表

// 添加元素
v.push_back(5);     // 尾部添加 O(1)
v.insert(v.begin(), 1);  // 头部插入 O(n)

// 删除元素
v.pop_back();       // 删除尾部
v.erase(v.begin()); // 删除第一个元素
v.clear();          // 清空所有

// 访问元素
int x = v[0];       // 不检查边界
int y = v.at(0);    // 检查边界，越界抛异常
int front = v.front();
int back = v.back();

// 容量相关
int size = v.size();
int cap = v.capacity();  // 当前容量
v.reserve(100);          // 预分配容量
v.shrink_to_fit();       // 释放多余内存
```

### 2.2 遍历方法

```cpp
// 方法1：下标遍历
for (size_t i = 0; i < v.size(); ++i) {
    cout << v[i] << " ";
}

// 方法2：范围for（推荐）
for (int x : v) {
    cout << x << " ";
}

// 方法3：迭代器
for (auto it = v.begin(); it != v.end(); ++it) {
    cout << *it << " ";
}
```

## 3. list（双向链表）

```cpp
#include <list>

list<int> lst = {1, 2, 3};

// 两端操作
lst.push_front(0);   // 头部插入 O(1)
lst.push_back(4);    // 尾部插入 O(1)
lst.pop_front();     // 删除头部
lst.pop_back();      // 删除尾部

// 中间插入（需要迭代器）
auto it = lst.begin();
advance(it, 2);      // 移动迭代器
lst.insert(it, 99);  // 在第3个位置插入

// 特有操作
lst.sort();                    // 排序
lst.unique();                  // 去重（需先排序）
lst.merge(lst2);               // 合并有序链表
lst.reverse();                 // 反转
lst.remove(3);                 // 删除所有值为3的元素
lst.remove_if([](int n){ return n > 5; }); // 条件删除
```

## 4. deque（双端队列）

```cpp
#include <deque>

deque<int> dq = {1, 2, 3};

// 双端操作
dq.push_front(0);
dq.push_back(4);
dq.pop_front();
dq.pop_back();

// 随机访问（比vector略慢）
int val = dq[2];     // 支持下标访问
```

## 5. map（有序映射）

### 5.1 基本使用

```cpp
#include <map>

map<string, int> ages;

// 插入
ages["Alice"] = 25;
ages.insert({"Bob", 30});
ages.insert(pair<string, int>("Charlie", 35));

// 查找
if (ages.find("Alice") != ages.end()) {
    cout << "Found: " << ages["Alice"] << endl;
}

// 访问（注意：[]会创建不存在的键）
int age = ages["David"];  // 如果David不存在，会插入值为0

// 更好的访问方式
auto it = ages.find("Eve");
if (it != ages.end()) {
    cout << it->first << ": " << it->second << endl;
}

// 删除
ages.erase("Alice");
ages.erase(ages.begin());

// 遍历
for (const auto& [name, age] : ages) {  // C++17结构化绑定
    cout << name << " -> " << age << endl;
}
```

### 5.2 map vs unordered_map

```cpp
#include <unordered_map>

// 用法相同，但无序且O(1)查找
unordered_map<string, int> umap;
umap["apple"] = 5;

// 何时使用？
// map: 需要有序遍历，内存占用小
// unordered_map: 追求最快查找速度，不关心顺序
```

## 6. set（集合）

```cpp
#include <set>

set<int> s = {3, 1, 4, 1, 5};  // 实际存储：1,3,4,5

s.insert(9);
s.erase(3);

// 查找
if (s.count(4)) {      // count返回0或1
    cout << "Found" << endl;
}

if (s.find(5) != s.end()) {
    // 找到了
}

// 遍历（自动升序）
for (int x : s) {
    cout << x << " ";  // 1 4 5 9
}
```

## 7. 迭代器详解

```cpp
vector<int> v = {10, 20, 30, 40};

// 正向迭代器
for (auto it = v.begin(); it != v.end(); ++it) {
    cout << *it << " ";
}

// 反向迭代器
for (auto it = v.rbegin(); it != v.rend(); ++it) {
    cout << *it << " ";  // 40 30 20 10
}

// 常量迭代器（只读）
for (auto it = v.cbegin(); it != v.cend(); ++it) {
    // *it = 100;  // 错误！不能修改
}
```

## 8. 容器适配器

### 8.1 stack（栈）

```cpp
#include <stack>

stack<int> st;
st.push(1);
st.push(2);
st.top();    // 2
st.pop();    // 移除顶部
st.empty();
st.size();
```

### 8.2 queue（队列）

```cpp
#include <queue>

queue<int> q;
q.push(1);
q.push(2);
q.front();   // 1
q.back();    // 2
q.pop();
```

### 8.3 priority_queue（优先队列）

```cpp
#include <queue>

// 最大堆（默认）
priority_queue<int> pq_max;
pq_max.push(3);
pq_max.push(1);
pq_max.push(4);
// 顶部总是最大元素：4

// 最小堆
priority_queue<int, vector<int>, greater<int>> pq_min;
pq_min.push(3);
pq_min.push(1);
pq_min.push(4);
// 顶部总是最小元素：1
```

## 9. 性能对比表

| 操作 | vector | deque | list | set/map | unordered |
|------|--------|-------|------|---------|-----------|
| 随机访问 | O(1) | O(1) | O(n) | O(log n) | O(1)平均 |
| 头部插入 | O(n) | O(1) | O(1) | O(log n) | O(1)平均 |
| 尾部插入 | O(1)* | O(1) | O(1) | O(log n) | O(1)平均 |
| 中间插入 | O(n) | O(n) | O(1) | O(log n) | O(1)平均 |
| 查找 | O(n) | O(n) | O(n) | O(log n) | O(1)平均 |
| 迭代器失效 | 重新分配时 | 两端插入时 | 永不失效 | 插入不影响 | 重哈希时 |

* vector尾部插入均摊O(1)，但可能触发扩容

## 10. 选择指南

```cpp
// 根据需求选择容器
// 1. 需要快速随机访问 → vector
// 2. 频繁头部/尾部操作 → deque
// 3. 频繁中间插入/删除 → list
// 4. 需要键值对查找 → unordered_map（最快）或 map（有序）
// 5. 需要去重 + 有序 → set
// 6. 需要快速找到最大/最小 → priority_queue
// 7. 固定大小 → array
```

## 11. 常见陷阱

### 11.1 迭代器失效

```cpp
vector<int> v = {1, 2, 3, 4, 5};

// ❌ 错误：插入可能导致迭代器失效
for (auto it = v.begin(); it != v.end(); ++it) {
    if (*it == 3) {
        v.insert(it, 99);  // it可能失效！
    }
}

// ✅ 正确做法
for (auto it = v.begin(); it != v.end(); ) {
    if (*it == 3) {
        it = v.insert(it, 99);  // 获取新的有效迭代器
        ++it;  // 跳过刚插入的元素
    }
    ++it;
}
```

### 11.2 map的[]操作符陷阱

```cpp
map<string, int> m;

// ❌ 意外创建元素
if (m["key"] == 0) {  // 如果"key"不存在，会被创建并赋值为0
    // 永远执行
}

// ✅ 正确查找
if (m.find("key") != m.end()) {
    // 元素存在
}
```

## 12. 实用示例

### 12.1 统计单词频率

```cpp
#include <iostream>
#include <map>
#include <sstream>

string text = "hello world hello cpp world hello";
map<string, int> freq;
stringstream ss(text);
string word;

while (ss >> word) {
    freq[word]++;
}

for (const auto& [w, count] : freq) {
    cout << w << ": " << count << endl;
}
// 输出：
// cpp: 1
// hello: 3
// world: 2
```

### 12.2 合并两个有序vector

```cpp
vector<int> v1 = {1, 3, 5, 7};
vector<int> v2 = {2, 4, 6, 8};
vector<int> result;

merge(v1.begin(), v1.end(), 
      v2.begin(), v2.end(), 
      back_inserter(result));
// result: 1 2 3 4 5 6 7 8
```