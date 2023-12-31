---
tags: msys2, C++, Windows, CMake, Clangd, VSCode, completed
---
- 培训时间：/
- 讲述人：刘与铮
- 培训内容：msys 2+cmake+clangd+vscode 功能介绍与环境搭建
- 培训目标：按照流程在自己电脑上完成 cpp 环境的搭建工作，并开始 cpp 语言的学习

首先，欢迎大家了解 RM 视觉组并学习一些相关知识。在往期培训过程中，我们一般使用 linux 进行环境配置，但是由于需要了解的知识过于硬核，导致培训效果不太理想，因此今年打算先使用 windows 配置环境并学习核心知识，之后再进行环境的过渡，徐徐图之。

# 一些要了解的环境基础知识

## 集成开发环境（IDE）

如果你不是第一次接触编程的话，这个词语应该并不陌生。IDE 是开发者和程序员进行软件设计和开发所需的工具和设施的集合，它提供了可以编写和测试代码的环境。

对于 cpp 开发而言，现在目前较为主流的 IDE 有 jetbrains 的 clion，微软家的 visual studio 等。这里推荐微软的开源编辑器 visual studio code（vscode），安装插件后便可以实现 IDE 的强大功能。
## 环境变量

这是一个在任何操作系统中都比较重要的概念，无论是 windows，macos 还是 linux。这里先引用一个关于 windows 环境变量介绍的博客：[看懂什么是环境变量! - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/82011100) 

我认为要说明这个概念，应该从终端开始说起，不过就不扯那么多了。简单来说，应用程序 A 的运行依赖应用程序 B 的一些文件，比如说原神的运行依赖 vc++。那么我们就需要安装 vc++，但是原神启动的时候，怎么找到这个文件呢？这就需要将 vc++的路径添加到环境变量中，让原神启动器找到它（实际上 vc++不用我们手动添加环境变量，它本身就装在环境变量中）。环境变量就像是一个应用程序的住址登记表，某一个程序想要运行另外一个程序的时候，就可以通过查这个表来找到它。

# 搭建环境

对于 c++开发环境，windows 下可选择的环境还有 visual studio (vs)，dev-c++等，然而 vs 使用的编译器为 msvc，与 linux 中使用的 gcc 有较大出入，dev-c++界面过于丑陋并且开发效率较低，因此我们主要介绍 msys 2 作为包含 gcc 的工具库，以 vscode 作为代码编辑器，进行环境配置。这里放一份不错的教程：[给萌新的C/C++环境搭建攻略（VSCode和MSYS2） - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/401188789) 
## msys 2

### 下载与安装

这个是官网下载链接：[MSYS2](https://www.msys2.org/)，并且包含了简单的环境配置过程。安装包下载速度较慢，有代理会快很多。

安装过程就不细说了，选择安装文件夹，建议不要选在 C 盘，之后是正常的安装流程。勾选一个运行 msys 2，会看到这样一个界面（我目前用的是 win10）：

![[msys2-UCRT64.png]]

看到这个界面，说明安装没有问题。

### 环境变量

现在要告诉其他应用 msys 2 装在哪里，也就是要把 msys 2 添加到环境变量当中。

win 10/11：设置->系统->关于->高级系统设置->环境变量

![[环境变量.png]]

找到系统变量中的 Path，打开：

![[编辑环境变量.png]]

点击新建，把刚才安装 msys 2 的路径粘进去。我们只需要添加 ucrt 64 的环境，如图所示：

![[新建环境变量.png]]

确定之后，环境变量就 ok 了。

### 安装编译工具

msys 2 本质上是编译工具链的管理工具，我们还需要使用它来安装一些工具。

还是刚才打开的 ucrt 64 界面，先输入 `pacman -Syu` 后回车进行软件源更新。这个过程可能有些慢，可通过更换国内源解决，大家可查阅 msys 2 换源的相关资料。关于 ucrt 的介绍知乎老哥说的比较清楚了，大家可以了解一下。之后 msys 2 应该会重启，按个 Y 回车后，重新打开即可。

之后，输入安装指令：`pacman -S mingw-w64-ucrt-x86_64-toolchain`，遇到需要选择的部分一路回车就行。

然后之后要用到的 clangd 一把全给他装了：`pacman -S mingw-w64-ucrt-x86_64-clang mingw-w64-ucrt-x86_64-clang-tools-extra`。以上安装步骤大致需要 3 G 的硬盘空间。

如何验证你装的东西有没有用呢？打开终端试试。

终端长这样：

![[WindowsPowerShell.png]]

分别输入 gcc 以及 clangd 后得到如图输出，说明安装成功，并且环境变量生效：

![[验证gcc与clangd.png]]

## cmake

cmake 是一个用于组织 cpp 项目的工具，能够实现多个 cpp 文件的构建，找头文件的效率更高，并且我们以后要用的 ros 2 也使用 cmake 构建项目，有必要学习一下。

cmake 下载地址：[Download | CMake](https://cmake.org/download/)，我们用 64 位的 windows 安装包。
安装过程也没啥好说的，记着勾上环境变量：

![[安装cmake.png]]

再开个终端，试一下有没有：

![[验证cmake.png]]

这个样子就 ok 了。

## vscode

为什么将 vscode 放在最后面呢，这里想告诉大家的是，实际上使用什么 IDE 并不是很重要，在之后我们将会有大量的时间直接在终端当中运行 cmake，使用一个 IDE 仅仅用于提升我们的开发效率。vscode 和记事本没有什么本质区别，它只是附带了语法高亮、cmake 相关的插件工具，才使其有了 IDE 的性能，最重要的还是刚才搭建的编译工具链。如果有人只装了个 vscode，来问我为啥没法写代码，那我只能说十分甚至九分的若至了。

官网下载：[Visual Studio Code - Code Editing. Redefined](https://code.visualstudio.com/) 推荐 system installer 版本，获取管理员权限更容易。

安装过程也没啥要说的，记得把添加环境变量勾上就行。

虽然它只是个编辑器，不过还是要欢迎大家使用，它强大的插件功能足够我们使用它用各种语言编程，用到大学毕业不是问题。

点击左边的四个框框图标，开始装插件。在上面的搜索栏搜索就行，目前需要的是这些插件，在左边那一列中显示。其中 synthwave 是我比较喜欢的一个黑色主题，大家也可找找看自己喜欢的主题。此外，. NET 插件会自动下载一些东西，这个有可能因为网速问题下载失败，不过问题不大，目前不影响使用。

![[VSCode应用拓展.png]]

然后打开一个空文件夹后进入，勾选信任此作者，到了这个界面：

![[VSCode工作区.png]]

这里就要介绍一下 vscode cmake 插件的用法了。

ctrl+shift+p, 输入 cmake，选中 cmake: quick start

![[VSCode-CMake快速入门.png]]

之后的步骤我就不放图片了，选择 gcc 13 的编译器，输入 cmake_test 作为项目名称，创建 c++项目，创建 executable（可执行文件），最后会生成一些文件：

![[生成CMakeLists.png]]

其中 cmakelists 就是用于管理 cmake 项目的工具，而 main. cpp 就是默认生成的实例代码。关于 cmake 的知识我们之后再说，现在配环境，重点是能编译过。

直接点击下面的生成，会看到这样的结果：

![[CMake编译成功.png]]

三角符号就是运行，点击之后可以看到 hello 输出：

![[CMake编译运行.png]]

至此，cpp 开发与运行环境基本搞定，大家可以在此基础上学习 cmake 脚本的编写以及 c++的语法知识了。