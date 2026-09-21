// 应用层:兵种入口(composition root)模板。
// 用法:复制 apps/templates/ 为 apps/<new>/,改本文件注释、config.yaml,
//       再把 <new> 加入根 CMakeLists.txt 的 set(APPS ...) 与 apps/CMakeLists.txt 的 KNOWN_APPS。
//
#include <iostream>

using namespace std;

int main()
{
  cout << "Hello, World!" << endl;
  return 0;
}
