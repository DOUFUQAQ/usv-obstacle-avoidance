 2025-07-15 16:30:02
"shane007"邀请你加入了群聊，群聊参与人还有：谢立鹏@鹏特立科技有限公司、LLTTt

 2025-07-15 16:30:02


 2025-07-15 16:30:14
"shane007"修改群名为“PTL-0715-001 路径规划”

shane007(wxid_zw1wuj1qpiq622) 2025-07-15 16:30:17
@谢立鹏 @LLTTt @Nameless Fade 1

shane007(wxid_zw1wuj1qpiq622) 2025-07-15 16:30:23
大家聊下吧

 2025-07-15 16:30:43
"shane007"修改群名为“PTL-0715-001 路径规划 未下单”

shane007(wxid_zw1wuj1qpiq622) 2025-07-15 16:31:32
@Nameless Fade 先确认需求，确定能做之后，让客户下单

shane007(wxid_zw1wuj1qpiq622) 2025-07-15 16:31:35
下单之后开工

我 2025-07-15 16:31:44
可以

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 16:31:57
可以做吗

我 2025-07-15 16:32:04
把需求Word发我吧

shane007(wxid_zw1wuj1qpiq622) 2025-07-15 16:32:11
@谢立鹏 @LLTTt 技术可以做，让客户下单吧

shane007(wxid_zw1wuj1qpiq622) 2025-07-15 16:32:18
[合并转发|谢立鹏和yu]
  谢立鹏 2025-07-15 下午 4:06
  需求：写在word里了，主要涉及usv避障
  最好也了解一些和大语言模型相关的，避障中要用到的
  用vrx或者uuv_simulator都可以
  预算；报价

  谢立鹏 2025-07-15 下午 4:07
  在VRX项目中采用全局路径规划+局部路径规划，其中局部路径规划用到调用VLM的API检测障碍物，把VLM输出给局部规划器，然后根据效果看是否修改局部规划器，VLM只是用于检测获取障碍物信息。全局规划用A*算法，局部规划算法用DWA算法。



shane007(wxid_zw1wuj1qpiq622) 2025-07-15 16:32:25
在VRX项目中采用全局路径规划+局部路径规划，其中局部路径规划用到调用VLM的API检测障碍物，把VLM输出给局部规划器，然后根据效果看是否修改局部规划器，VLM只是用于检测获取障碍物信息。全局规划用A*算法，局部规划算法用DWA算法。

shane007(wxid_zw1wuj1qpiq622) 2025-07-15 16:32:30
轨迹优化部分采用B样条算法

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 16:32:41
需要我提供什么

我 2025-07-15 16:32:55
不是还有Word文档吗我看一下

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 16:33:36
[文件|usv避障.docx](http://127.0.0.1:5030/file/ef33e5c700a75107d361d192b07c4f51)

我 2025-07-15 16:34:58
OK

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 16:35:27
还有啥问题吗

我 2025-07-15 16:36:13
目前没什么了有问题我再反馈

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 16:38:25
好的，但是我有两点担心，一是调用VLM的时间，在局部规划中是否来得及，二是在仿真环境中，VLM没见过仿真物体，VLM通常认识的是真实世界中的物体

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 16:44:41
@Nameless Fade 

shane007(wxid_zw1wuj1qpiq622) 2025-07-15 16:46:28
@Nameless Fade 

我 2025-07-15 16:46:55
我先试一会，有问题我会及时反馈的

shane007(wxid_zw1wuj1qpiq622) 2025-07-15 16:46:58
@Nameless Fade 客户下单前，注意不要透露过多技术细节，防止客户偷方案

我 2025-07-15 16:47:08
好的

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 18:59:55
@Nameless Fade 

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 18:59:57
可以做吗

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 19:03:18
"LLTTt" 拍了拍我的zio时空驱动器

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 19:28:26
@Nameless Fade 

我 2025-07-15 19:28:37
可以的

我 2025-07-15 19:28:46
老哥你让客户下单吧

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 19:28:47
可以是吗

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 19:29:45
试成功了吗

我 2025-07-15 20:09:15
可以的

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 20:13:44
是在开源的VRX上面吧？

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 20:13:49
能麻烦你梳理一下开发功能点吗，到时候按照功能点来看

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 20:13:54
我是想要规划+优化+控制的

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 20:13:56
@Nameless Fade 

shane007(wxid_zw1wuj1qpiq622) 2025-07-15 20:14:07
@Nameless Fade 客户下单前，注意不要透露过多技术细节，防止客户偷方案

我 2025-07-15 20:28:53
> LLTTt(wxid_r1l2zsxp5zf612) 07-15 20:13:49
> 能麻烦你梳理一下开发功能点吗，到时候按照功能点来看
按照发的需求来的

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 20:30:59
咱们一开始最好都过一遍，剩的最后会因为我认为某些需要开发，你认为不用开发的

shane007(wxid_zw1wuj1qpiq622) 2025-07-15 20:31:53


shane007(wxid_zw1wuj1qpiq622) 2025-07-15 20:32:09
@Nameless Fade 第一件事，首先推动客户下单，之后再逐步推进细化

我 2025-07-15 20:32:11
嗯嗯，所以老哥你让客户给我完整的需求都写好清楚给我就好了，我会按着需求来完成的

shane007(wxid_zw1wuj1qpiq622) 2025-07-15 20:32:42
@LLTTt 让客户下单吧

shane007(wxid_zw1wuj1qpiq622) 2025-07-15 20:32:53
这边技术没有问题的，可以做

shane007(wxid_zw1wuj1qpiq622) 2025-07-15 20:33:06
下单之后，逐步推进细节

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 20:33:20
好我再把需求完善下

shane007(wxid_zw1wuj1qpiq622) 2025-07-15 20:33:31
别到时候一句话，找个借口说不做了，大家白忙活

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 20:40:41
这样你就先完成全局路径规划加上采用调用VLM的局部规划。先完成这个需求，再搞后续的优化和MPC控制。怎么样

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 20:40:48
"LLTTt" 拍了拍我的zio时空驱动器

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 20:41:49
你先用简单的pid跟踪上规划出来的轨迹，后续再找你进行优化和高级控制算法的开发

shane007(wxid_zw1wuj1qpiq622) 2025-07-15 20:44:27
@Nameless Fade 

shane007(wxid_zw1wuj1qpiq622) 2025-07-15 20:45:31
@Nameless Fade 你回答下问题吧

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 20:56:20
[文件|usv避障.docx](http://127.0.0.1:5030/file/9acf0b5f7ad7a54ea1f91e6b6ddb0a57)

shane007(wxid_zw1wuj1qpiq622) 2025-07-15 20:58:21
![图片](http://127.0.0.1:5030/image/7cd68d2a4718e9cab2579daacddd9da3)

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 21:58:18
@Nameless Fade 

我 2025-07-15 22:04:16
> wxid_r1l2zsxp5zf612 07-15 16:38:25
> 好的，但是我有两点担心，一是调用VLM的时间，在局部规划中是否来得及，二是在仿真环境中，VLM没见过仿真物体，VLM通常认识的是真实世界中的物体
对于vlm的时间问题，可以采用​​异步处理和帧采样优化，或者采用蒸馏后的轻量vlm模型来解决，vlm没见过仿真物体的问题可以使用合成数据微调​​来解决

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 22:54:03
光做规划部分是多少

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 22:54:05
@Nameless Fade 

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 22:54:13
到手就行

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 22:56:51
@Nameless Fade 

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 23:48:43
@Nameless Fade 

shane007(wxid_zw1wuj1qpiq622) 2025-07-15 23:49:26
@Nameless Fade 

我 2025-07-15 23:49:54
> LLTTt(wxid_r1l2zsxp5zf612) 07-15 22:54:03
> 光做规划部分是多少
价格吗还是什么

25984983067141610@openim 2025-07-15 23:50:03
价格

shane007(wxid_zw1wuj1qpiq622) 2025-07-15 23:52:03
@Nameless Fade 你报总价即可

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 23:52:06
多少呢

我 2025-07-15 23:53:58
等会我先评估一下

我 2025-07-15 23:54:44
就不做仿真部分了对吧

我 2025-07-15 23:56:00
1300这样吧

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-15 23:58:27
> Nameless Fade(wxid_kjvd4skd92a022) 07-15 23:54:44
> 就不做仿真部分了对吧
啥意思

我 2025-07-15 23:59:17
不是说只做规划部分吗？

我 2025-07-15 23:59:30
全部一起的话就还是之前的价格

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 00:00:03
规划不用在仿真中验证码

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 00:01:52
吗

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 00:01:58
"LLTTt" 拍了拍我的zio时空驱动器

25984983067141610@openim 2025-07-16 00:08:53
@Nameless Fade 

shane007(wxid_zw1wuj1qpiq622) 2025-07-16 00:15:55
> LLTTt(wxid_r1l2zsxp5zf612) 07-16 00:00:03
> 规划不用在仿真中验证码
对，技术已经报价1300

shane007(wxid_zw1wuj1qpiq622) 2025-07-16 00:15:59
你问下行不行

我 2025-07-16 00:16:22
> LLTTt(wxid_r1l2zsxp5zf612) 07-16 00:00:03
> 规划不用在仿真中验证码
要的，但是后面交付的只有算法

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 00:16:55
那你规划怎么验证，我没太懂

我 2025-07-16 00:18:03
> LLTTt(wxid_r1l2zsxp5zf612) 07-15 20:56:20
> [文件|usv避障.docx](http://127.0.0.1:5030/file/9acf0b5f7ad7a54ea1f91e6b6ddb0a57)
这个一整个完整的需求就是之前的价格1500

我 2025-07-16 00:18:41
我会交付算法，但是验证部分的结果我不会交付

shane007(wxid_zw1wuj1qpiq622) 2025-07-16 00:19:34
> LLTTt(wxid_r1l2zsxp5zf612) 07-15 22:54:03
> 光做规划部分是多少
问下客户，这句话啥意思，比最初的需求去掉了哪些部分？

shane007(wxid_zw1wuj1qpiq622) 2025-07-16 00:19:46
@LLTTt 

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 00:20:11
验证部分的结果包括什么

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 00:20:15
全包1500对吗

我 2025-07-16 00:20:26
> LLTTt(wxid_r1l2zsxp5zf612) 07-16 00:20:15
> 全包1500对吗
对的

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 00:20:36
> LLTTt(wxid_r1l2zsxp5zf612) 07-16 00:20:11
> 验证部分的结果包括什么
1

我 2025-07-16 00:21:11
> LLTTt(wxid_r1l2zsxp5zf612) 07-16 00:20:11
> 验证部分的结果包括什么
仿真结果

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 00:22:10
实验结果包含还是不包含

我 2025-07-16 00:22:44
不包含

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 00:23:50
也就是你只有实现，算法对比是我做的部分是吧

我 2025-07-16 00:24:40
我拍了拍自己的zio时空驱动器

我 2025-07-16 00:25:08
对，需求书里面所有的话就是1500

我 2025-07-16 00:25:42
我建议的话还是完整的需求吧省的你也麻烦我也麻烦

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 00:26:51
这个是包含实验验证的吧？

我 2025-07-16 00:27:19
对的

我 2025-07-16 00:27:25
完整的需求

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 00:28:29
明白你说的完整需求就是1500把我规划以及的规划实验全做完，包含仿真结果是吧

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 00:29:25
行，我再考虑一下，今天也比较晚了

我 2025-07-16 00:31:45
> LLTTt(wxid_r1l2zsxp5zf612) 07-16 00:28:29
> 明白你说的完整需求就是1500把我规划以及的规划实验全做完，包含仿真结果是吧
对的

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 00:35:20
明天下吧，一会睡了

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 08:23:52
时间是多久完成

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 08:23:58
"LLTTt" 拍了拍我的zio时空驱动器

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 08:24:01
@Nameless Fade 

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 08:29:25
最后做完，得证明加入VLM效果确实提升了哈

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 08:29:31
仿真结果也给我

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 08:33:05
还有也需要向我展示并解释用法，方便我写小论文 都能完成就可以下单了

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 08:33:11
@Nameless Fade 

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 08:37:48
你要避障的话，避障场景也需要设计一下，都没问题就可以下单了 

shane007(wxid_zw1wuj1qpiq622) 2025-07-16 08:38:13
@Nameless Fade 

我 2025-07-16 09:52:28
OK的

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 15:29:53
调用VLM的api也能微调吗

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 15:30:00
"LLTTt" 拍了拍我的zio时空驱动器

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 15:39:55
已经下单了

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 15:40:00
麻烦回复下 

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 15:40:03
@Nameless Fade 

 2025-07-16 15:40:11
"LLTTt"修改群名为“PTL-0715-001 路径规划 已下单”

shane007(wxid_zw1wuj1qpiq622) 2025-07-16 15:58:43
@Nameless Fade 客户已经下单，开工吧！

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 16:53:39
@Nameless Fade 

我 2025-07-16 17:00:37
> shane007(wxid_zw1wuj1qpiq622) 07-16 15:58:43
> @Nameless Fade 客户已经下单，开工吧！
OK收到

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 18:21:51
还有什么问题吗

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 18:21:52
@Nameless Fade 

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 18:21:57
> LLTTt(wxid_r1l2zsxp5zf612) 07-16 15:29:53
> 调用VLM的api也能微调吗
1

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 18:32:16
@Nameless Fade 

我 2025-07-16 18:32:41
> LLTTt(wxid_r1l2zsxp5zf612) 07-16 15:29:53
> 调用VLM的api也能微调吗
在本地微调

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 18:41:26
本地微调，调用的是本地的vlm吗

shane007(wxid_zw1wuj1qpiq622) 2025-07-16 18:57:21
@Nameless Fade 

shane007(wxid_zw1wuj1qpiq622) 2025-07-16 18:57:54
@Nameless Fade 回复下吧

我 2025-07-16 19:00:30
> LLTTt(wxid_r1l2zsxp5zf612) 07-16 18:41:26
> 本地微调，调用的是本地的vlm吗
api是可以本地微调调用的

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-16 19:02:14
好的，你开始整吧

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-18 07:12:57
技术老师，开发顺利吗

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-18 14:50:18
@Nameless Fade 

我 2025-07-18 14:54:05
> LLTTt(wxid_r1l2zsxp5zf612) 07-18 07:12:57
> 技术老师，开发顺利吗
正常进行中

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-19 09:37:43
@Nameless Fade 

LLTTt(wxid_r1l2zsxp5zf612) 2025-07-19 09:37:57
麻烦发下进度，截图啥的，做到哪里了

