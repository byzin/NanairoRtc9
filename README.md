# NanairoRtc9

The path tracing renderer for RTC9.

# 環境要件

* C++20をサポートするコンパイラ
    - Visual Studio 2022
    - Clang 16 以降
    - GCC 13 以降
* Git
* [CMake](https://cmake.org/ "CMake"): 3.27
* [Python](https://www.python.org/ "Python"): 3
* [Vulkan SDK](https://vulkan.lunarg.com/ "Vulkan"): 1.3.261
    - *Vulkan Memory Allocator header* 込でインストールしてください

## 任意

以下のツールは入れておくと *Nanairo* のビルド時間を短縮できます

* [Clspv](https://github.com/google/clspv "clspv"): 開発中のためmasterブランチを使用

# ビルド

## サブモジュールのダウンロード

```
git submodule update --init --recursive
```

## CMake

cmakeのビルドは `build` ディレクトリで行います。

### ClspvがPATHに無い場合

ClspvがPATHにある場合はNanairoはそれを見つけて使います。
もしClspvが無い場合は以下のcmakeコマンドに `-DZIVC_USES_OWN_CLSPV=ON` を足して実行してください。
時間はかかりますがNanairoはClspvを内部でビルドしてそれを使用します。

### Visual Studio MSVC

ここではコマンドライン (e.g. `C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat`) からcmakeを実行する方法を記述します。
GitやCMake、PythonのPATHを設定して実行してください。

```
cmake -G"Visual Studio 17 2022" -Ax64 -Thost=x64 -DCMAKE_BUILD_TYPE=Release -DZIVC_BAKE_KERNELS=ON -DZ_ENABLE_HARDWARE_FEATURES=ON ..
```

### Ubuntu GCC

```
CC=gcc-13 CXX=g++-13 cmake -DCMAKE_BUILD_TYPE=Release -DZIVC_BAKE_KERNELS=ON -DZ_ENABLE_HARDWARE_FEATURES=ON ..
```

### Ubuntu Clang

```
CC=clang CXX=clang++ cmake -DCMAKE_BUILD_TYPE=Release -DZ_CLANG_USES_LLVM_TOOLS=ON -DZIVC_BAKE_KERNELS=ON -DZ_ENABLE_HARDWARE_FEATURES=ON ..
```

## ビルド実行

### Windows

```
cmake --build . --config Release --parallel -- -noLogo -v:minimal -clp:Summary
```

### Ubuntu

```
cmake --build . --config Release --parallel
```

# 実行

```
ctest -C Release -V -R Nanairo-single-vulkan
```

画像は test ディレクトリに出力されます。
