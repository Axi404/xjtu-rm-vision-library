
# 常见问题

1. fatal: unable to access 'https://github.com/xxx/xjtu-rm-vision-library.git/ ': Recv failure: Connection was reset

问题描述： 这是由于网络连接问题，也可能是端口不对，可以通过在git中手动配置的方法
解决方案： 
	先查看自己的端口号  ``Port : xxx
	然后在 git bash 中输入以下命令
	``git config --global http.proxy http://127.0.0.1:xxx
	xxx 指的就是上述查询到的端口号

2. 创建了一个错误的仓库 想要删除
	先点击仓库内的``settings`` ，拖拉至页面最下方，点击 ``Delete this respository`` ，再点击 ``I have read and understand these effects``（建议认真阅读该提示）后，将上方仓库名复制到对话框后，点击 ``Delete this repository``。恭喜你，已经在删库跑路的大道上迈出了一大步！
