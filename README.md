# 已有的权值更新策略

## 依上下游关系，根据需求更新
1. 工厂-工厂：上下游关系(BC类)
    1. 下游有无需求？(BC类)
        1. 有：上游有无产品？
            1. 有：-∞
            2. 无：是否在生产：
                1. 在生产：+∞
                2. 没生产：0
        2. 无：-∞
2. 机器人(可作为A类的上游)-工厂(AD类)
    1. 工厂为A类？
        1. 是：有无机器人正在派送
            1. 有：+∞
            2. 无：d
    2. 工厂为D类？
        1. 是：+∞


## 机器人-工厂权值更新：不应依赖于机器人是否空闲（注意：这里会覆盖依赖需求的更新
1. 机器人-工厂：依靠是否耽搁去购买产品
    1. 有没有人去送产品？
	    1. 有：+∞
	    2. 无：工厂有无产品？
		    1. 有：二者间距离d
		    2. 无（认为1，2，3始终有产品）：根据dt x 速度(可调节) x 剩余生产帧数k是否大于d
			    1. 大于：+∞
			    2. 小于：d

## 任务分配时：只对空闲分配（待补充
1. 所有机器人-D类：+∞
2. 空闲机器人计算最短路径
    1. 按顺序计算最短路径
    2. 按得到的路径分配任务，
            1. 把任务目的地到所有同类型出发地的权值置为+∞
        1. 将所有机器人到出发地权值设为+∞

# 特殊情况：
1. 机器人到工厂途中，工厂依旧有产品，因而其他机器人在下一帧也可能去这里
    * 解决办法：2.2.2
2. 工厂生产不均匀
    * 解决办法：（未解决）
        1. 在456间引入时间系数，计算方式：k+=剩余t若不生产，则k+=-T/2
        2. 将时间系数乘在对应类型工厂到7之间
3. 如果不对A类限制，所有机器人一开始都会跑到某个A类去
    * 解决办法（未解决）：工厂-工厂广播更新时，加入A类进入判断，之前所有A类都要continue
    * 问题：之前写的函数只考虑了BC类，需要针对性考虑A类，否则容易有map越界的错误
        * 解决办法：判断为A类后，直接根据该id的工厂对应仓库的第二个状态判断有无机器人正在派送
            * 有：+∞
            * 无：d
4. 剩余生产时间t和是否有产品的问题：可以通过判断dt x 速度(可调节) x t是否大于当前距离d
    * 小于：冲过去不会影响拿产品，可以忽略有无产品的状态
    * 大于：冲过去会影响拿产品，不能忽略有无产品的状态
    * 解决办法：1.2.2


# 机器人任务分配策略
根据传入的机器人即将进行购买-销售的路线节点
1. 锁定购买材料节点生产的产品，即SetProduceFlag(true)
2. 锁定销售材料节点对应的仓库，即SetWarehouseFlag(buyNodeType, true)
   
  * 特殊情况是：对于D类工作台的需求是一直打开的，即SetWarehouseFlag(buyNodeType, false)
  