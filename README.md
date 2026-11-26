宠辱不惊，一直努力，做好自己的本分，就是很不错的人。
这些年失败的经历主要是一直不敢做出自己的选择，总是认为时机还不成熟，希望等自己能力更强之后再去追求，但是后来要不就是时过境迁伊人不在，要不就是自己也逐渐淡忘了自己的目标变得浑浑噩噩。


这份清单围绕机械臂到达、轨迹跟踪和简单操作，回答每块负责什么、输入输出是什么、学哪些原理、跟哪些章节、学到什么粒度，以及如何验收和串联。九大编号便于对照，**不是机器人处理顺序，也不是严格学习顺序**。“核心必学”指这条 Robot Learning 学习路线需要建立的知识，不代表每个机器人任务都必须用齐九块。

前四模块的理论资源已经选定：跟首尔大学 Frank C. Park 的手推课程，以《Modern Robotics》章节定位。第2–3章对应空间表示；第4、5、6章分别对应 FK、Jacobian、IK；第8、11章对应动力学与控制；第9、10章对应轨迹与规划。具体播放列表尚未提供，不能据教材目录声称所选视频完整覆盖。官网的带解练习与首尔大学历年题，可配合“手推—独立实现—数值或物理验证”使用。[1] [2]

每块把**完整学习深度**和**最初可验收实验**分开：先得到可靠反馈，再继续系统学习，不把小实验当作知识终点，也不要求看完所有课程才开始实践。课程和作业承担不同角色；Cornell、ETH 是具体练习入口，不是固定唯一主线。

## 先把系统闭环与 action 说清楚

可以用六问检查系统：如何接收信息、如何变成状态、如何生成动作、如何到执行器、如何判断成功、如何定位失败。这里仅借用原文章的技术闭环，不采用其职业或商业判断。[3]

**机械臂桌面目标闭环：**图像与关节反馈 → 视觉测量 → 可选的概率融合 → 基座坐标下的目标与状态 → IK、轨迹和位置控制，或学习策略 → 驱动执行 → 新反馈。目标点还不是完整抓取位姿；接近方向、夹爪朝向、物体高度和接触条件需要另行定义。端到端视觉策略可以不显式输出位姿，但观测时序、动作语义和底层安全约束不会消失。

**阿克曼车沿车道闭环：**车道像素 → 内外参与平地约束 → 横向、航向偏差测量 → 若传感器可用，结合里程计、IMU 与运动模型估计状态 → 跟踪控制产生转角和速度 → 反馈。也可以直接用画面误差循迹，不必先建立完整米制几何；但这时不能自动声称获得米制全局位置。全局定位与 SLAM 是另一层需求，不是所有循迹任务的前置。

感知和 RL 都不必须使用神经网络。映射可以由几何公式、查表、控制算法或网络实现。RL 是学习序列决策的方法；网络是策略或价值函数的一种表示。表格 MC、TD 不依赖深度学习；DQN 网络输出各动作的 Q 值，再按规则选动作；actor 可以输出约定动作或动作分布。[4]

每次实验都写清 action 是关节目标、末端增量、速度还是力矩，不能只标“六维向量”。MDP 的 state 应包含预测下一步转移所需的信息。这里主要讨论物理与任务状态，和业务状态机中的阶段标签不是同一层次；有影响的任务阶段也可以成为状态变量。机器人常只有带噪的部分 observation，代码里的 state 向量也未必具有严格 Markov 性。

## 1. 空间表示：位置、姿态与坐标变换【路线核心】

**职责与输入输出。**统一“在哪里、朝向哪里”的表达。输入点、位姿与坐标系关系，输出另一个坐标系下的点或位姿，供相机、运动学和策略共享。

**课程与范围。**Frank Park 手推配《Modern Robotics》第2章的构型、自由度与构型空间，第3章 Rigid-Body Motions 的刚体位姿。主干是二维/三维旋转、SO(3)、SE(3)、齐次变换、轴角、指数坐标和 twist，以及 spatial/body 表示。[1]

**原理粒度。**能区分物体移动与换坐标系，解释主动旋转和被动变换；独立推导旋转矩阵正交性、变换的逆、组合顺序，以及固定旋转轴或螺旋轴的指数变换。每个矩阵都注明从哪个坐标系映射到哪个坐标系，不能靠乘法“试对”。

**最初实践验收。**用 NumPy 写二维旋转、变换组合、逆和点变换；再随机生成三维位姿，检查正交性、行列式与往返误差。同一点经两条等价变换链应一致，数值容差须声明；故意颠倒顺序，解释错误出现在哪里。

**依赖与深入。**线代、微积分与5A即可起步，直接接模块2和相机外参。继续深入李群、李代数及姿态插值；闭链和复杂移动机器人构型不是这个实验的前置。

## 2. 运动学：FK、Jacobian、IK 与可达性【路线核心】

**职责与输入输出。**连接关节空间与末端空间：关节构型经 FK 得到末端位姿，目标位姿经 IK 得到可能的关节解，Jacobian 映射局部速度。它不保证真实机械臂按时到达。

**课程与范围。**《Modern Robotics》第4章正运动学、第5章速度运动学与 Jacobian、第6章逆运动学。学习零位构型、关节螺旋轴、指数积 POE、spatial/body Jacobian，以及解析和数值 IK。DH 可作为另一种建模方式对照。[2]

**原理粒度。**手推两连杆 FK、Jacobian 和解析 IK，再理解三维 POE、位置/姿态误差与迭代求解。能用 SVD 说明奇异值、不可实现方向和伪逆的噪声放大，推导阻尼最小二乘。目标有位置解，不表示姿态、限位或到达路径也可行。

**最初实践验收。**实现两连杆模型，保留可达、不可达、近奇异三类目标；用有限差分验证 Jacobian，比较解析与数值 IK。扫描初值，记录位置误差、姿态误差、迭代次数和限位情况。不可达、多解与数值不收敛应分别报告，而非一律增加迭代次数。

**依赖与深入。**依赖模块1与5A，接模块4的参考和策略动作约束。完整学习可继续到三维 POE、受限 IK、冗余与零空间任务；复杂闭链和接触约束随后补。

## 3. 动力学与反馈控制【路线核心；真机受接口限制】

**职责与输入输出。**解释力如何改变运动，以及反馈如何抵抗偏差。输入模型、参考位置/速度和当前反馈，输出力矩或执行器允许的控制量；位置伺服接口下，用户发送的关节目标不等于亲自实现底层力矩控制。

**课程与范围。**《Modern Robotics》第8章开链动力学、第11章机器人控制；ETH HW2 可作控制与 MDP 的补充实践。[1] [5] 主干是 `M(q)q̈+c(q,q̇)+g(q)=τ`，惯量、科氏/离心项、重力和外力，正/逆动力学，以及 PD、PID、重力补偿、计算力矩、采样、饱和与积分累积。

**原理粒度。**从拉格朗日或牛顿—欧拉框架独立推导小模型，检查量纲和能量关系；解释比例增益、微分阻尼、积分与饱和如何改变超调、振荡和稳态误差。控制不是让动画“看起来能动”，而是说明参考、误差、输入与响应之间的因果关系。

**最初实践验收。**先在仿真实现二连杆或倒立摆动力学与离散积分；同初值、同参考比较无控制、重力补偿、PD、PID，记录误差、超调、稳态偏差和控制输入。改变采样周期、负载与输入限幅，区分模型误差、增益问题和执行器饱和，保留稳定基线。

**依赖与深入。**依赖模块1–2、5A和常微分方程；完整控制体系不是开始 BC 的前置。继续学习离散稳定性、鲁棒/自适应控制，再深入 MPC。SO101 候选平台可验证位置伺服响应，但完整小模型动力学和自写力矩控制先在仿真验收，边界见模块7。

## 4. 路径、轨迹生成与运动规划【基础核心；复杂规划按需】

**职责与输入输出。**决定如何经过、何时到达。输入起点、目标、障碍和约束，输出可行路径及带时间的参考位置、速度、加速度，交给控制层执行。

**课程与范围。**《Modern Robotics》第9章轨迹生成、第10章运动规划，实践可继续用 ETH HW2。轨迹生成不是完整运动规划：路径描述几何，轨迹给路径加时间。学习关节/笛卡尔路径、时间标定、边界条件、构型空间和碰撞约束，再理解采样规划结构。[1] [5]

**原理粒度。**独立推导五次多项式满足起止位置、速度和加速度的系数。解释相同路径加快后，速度、加速度和力矩需求为何变化；理解规划器找出路径不代表控制器能以任意速度执行。需要障碍规避时再实现 RRT 等方法。

**最初实践验收。**先做平滑点到点轨迹，再生成“∞”形末端路径，经 IK 转为连续关节参考并跟踪。分别画参考路径、实际轨迹与误差，比较执行速度，检查 IK 分支跳变、限位和饱和。加入障碍后验证整段路径，不只检查终点。

**依赖与深入。**模块2先提供几何可行性，模块3验证动态跟踪，模块7记录响应；输出可以成为模块8的专家数据。完整学习再进入轨迹优化、动态障碍与最优控制，不把它们全部设为模仿学习入口门槛。

## 5. 编程与深度学习：分清实验工具和学习模型

### 5A. Python / NumPy【前四模块实验就用】

**职责与输入输出。**把数学模型变成可复现计算。输入模型参数、数组和实验配置，输出数值结果、曲线与日志；这部分不需要神经网络。

**课程与范围。**Cornell A0 是 Python/NumPy/PyTorch 能力检查，也可用等价课程练习验收，不强迫重复已会的任务。[6] Python 补函数、模块、文件读写和环境管理；NumPy 补索引、广播、矩阵运算、线性求解、SVD和绘图。

**原理粒度与最初验收。**能解释 shape、轴、单位与数值误差，把前四模块的小模型独立实现。一次广播或维度错误应能定位，固定参数与初值后结果可复现；不以“会调用库”替代推导。

**依赖与深入。**与模块1–4并行。后续根据数据规模补向量化、测试和性能分析，不先把工程工具全部学完。

### 5B. 深度学习原理与 PyTorch【学习型感知、策略或价值函数需要时】

**职责与输入输出。**把数据拟合成函数。训练输入观测/状态、目标和标签，输出参数；执行时输出检测结果、动作或价值，具体语义由任务定义，模型不替代闭环评价。

**课程与范围。**三蓝一棕2017年神经网络基础四集作导入，内容包括网络、梯度下降和反传，其反传微积分会推链式法则，不只是浅层图示。[7] [8] 吴恩达本人主讲的 DLS 前两门作系统原理与关键作业：第1门 Neural Networks and Deep Learning，第2门正则化、优化、调参与 Batch Normalization。DLS 实践包含 NumPy/Python/TensorFlow，不是 PyTorch 版。[9]

框架实践用 Laurence Moroney 主讲的 DeepLearning.AI《PyTorch for Deep Learning》第1门 Fundamentals：Getting Started、Workflow、Data Management、Core Neural Network Components。第2门调参与 TorchVision 按任务补，文本任务和第3门高级架构/部署后期再学；查用配 PyTorch Learn the Basics，不机械重复同类作业。[10] [11] [12]

**原理粒度。**能推两层 MLP 前向、MSE 梯度、链式法则和参数更新，解释 loss、学习率、正则化与过拟合。理解 batch、feature、时间维，以及 dtype/device、计算图与清梯度。`backward()`成功运行不是原理验收。

**最初实践验收。**先用 NumPy 写小 MLP 和手动梯度，以有限差分核对一个参数；再用 PyTorch 完整完成数据加载、train、validation、保存最佳权重和重载。数据按整条轨迹划分，防止相邻帧泄漏；测试集不参与选参，保存可复现训练曲线。动作标签可来自前四模块基线，但离线预测准不等于闭环成功。

**依赖与深入。**5A、线代、链式法则与概率期望。接学习型视觉、BC或深度 RL；CNN、复杂策略表达与大型训练按需求深入，几何感知和表格 RL 不因此被延后。

## 6. 视觉测量与概率状态估计：两条相连但不同的主干

模块6原本包含太多内容，不能当成与一门 RL 课同等体量的单课。6A 内部还有图像找点/处理、空间几何、动态视觉三个层次；6B 负责概率表示和动态融合。光流、SfM、视觉里程计也估运动，但不等于完整多传感器状态估计。

### 6A. 视觉测量：像素如何约束空间【视觉任务核心】

**职责与输入输出。**输入图像、内外参以及深度、已知平面或多视图约束，输出点、位姿、相对运动或结构测量，带坐标系、单位、时间戳和有效标志。检测负责在图像中找什么，几何负责它在空间中意味着什么。

**课程与范围。**宾大《Robotics: Perception》作为条件主课程，四模块是 Geometry of Image Formation、Projective Transformations、Pose Estimation、Multi-View Geometry。依次学习相机/透视/内参/标定，齐次坐标/单应性/单视图测量，SVD/最小二乘/RANSAC/3D对应/P3P，再到极线几何、光流、视觉里程计和 Bundle Adjustment。[13] [14]

公开学习者仓库记录 Dolly Zoom、Homographies、Augmented Reality、SfM 四个 MATLAB 项目。先按原作业理解，再选1–2个关键实验用 NumPy 重做，利于验证算法而不是同时重写所有工具。仓库不是官方可用保证，当前课程与作业稳定获取尚未确认；按英语课程准备，字幕与 MATLAB 授权需实际确认，不假定 Octave 兼容全部工具箱。这些条件需要解决，但不据此否决课程体系。[15]

Nayar 的 First Principles of Computer Vision 用于补直觉、几何基础和深入专题，具体范围如下，不要求先学完全部 CV：[16]

- **Image Formation FPCV-1-1**：针孔与透视投影是主干；**Image Sensing 1-2** 的传感基础按需补。[17]
- **Image Processing I 1-4**：卷积，高斯、中值、双边滤波及模板匹配基础；**II 1-5**：Fourier、低/高频、卷积定理与采样混叠基础。复杂去模糊后期深入。[16]
- **Edge Detection 2-1** 与特征检测/匹配基础：用于稳定测量；**SIFT 2-3** 可深入，**Image Stitching 2-4** 按项目补。[16]
- **Camera Calibration 4-1**：几何定位核心；**Uncalibrated Stereo 4-2**：任务需要深度或多视图时深入。[18] [19]
- **Optical Flow 4-3** 与 **Structure from Motion 4-4**：纳入动态视觉后续深入。前者推亮度恒定、小位移、一阶泰勒、光流约束、孔径问题与 Lucas–Kanade 最小二乘；输出二维表观运动，不直接是真实三维速度。后者联合恢复场景结构与相机运动，该讲主要是正交投影近似和 Tomasi–Kanade 因子分解，不是现代透视 SfM 完整工程。[20] [21]
- **Depth from Defocus 3-4**：先懂光学原理，有专门测深需求再推导和实践。它依靠不同光学设置下的模糊与已知模型求深度，不能概括成“越模糊越远”。其他专门重建按项目补。[16]

Reconstruction II 是模块名，包含4-1至4-4，不是整套课程的“第二课”。二维光流不必先修标定或双目；深入 SfM 需要投影、对应和矩阵基础，特征跟踪也不强制只用光流。Bonn Photogrammetry I/II 可在标定、几何或后续估计难点上替代/补充；Robotics Book 5.3.3–5.3.8 可作成像查用，不再列候选排行榜。[22] [23]

**原理粒度。**能推 `u=fxX/Z+cx, v=fyY/Z+cy`，理解畸变、内参 K、外参 R/t与重投影误差。对去畸变后的像素，`K⁻¹[u,v,1]ᵀ`只给射线；恢复单点还需轴向深度、已知平面或其他几何信息。相机标定不会自动提供深度；两台相机也不会自动成为校准立体系统，还需同步、标定、对应与三角化。

机器人零位标定确定关节读数与运动学模型的一致性；相机内参标定确定成像参数；外参标定连接相机、桌面或机器人基座坐标系。手眼标定则解决相机与机器人之间的固定变换关系，具体形式取决于相机装在末端还是固定在外部。它们解决的问题不同，不能相互替代。

**最初实践验收。**先用合成投影、手工选点或棋盘，让坐标问题独立验证，再加检测。多姿态棋盘标定输入真实方格尺寸，去畸变后做已知平面求交，并把结果反向投影；PnP 可用已知3D–2D对应验证位姿。[24] [25] 用未参与拟合的已知点报告位置误差，同时报告重投影误差，不能只看叠加图“差不多”。

桌面标记与物体顶部/中心必须分开：未知高度的实物不能把射线与桌面交点当正确位置。故意抬高标记，解释平面假设何时失效；固定相机外参转到基座后，先不驱动机械臂检查轴向、尺度和可达性。一个点仍不是完整抓取位姿。

**依赖与深入。**模块1、5A、最小二乘即可起步，不依赖完整动力学或 RL。完整深度继续到鲁棒对应、双/多视图、光流与 SfM，必要时进入视觉里程计或 SLAM；学习型检测按需求接5B。动态视觉有价值，但不是固定桌面定位第一个实验的前置。

### 6B. 概率状态估计：历史与模型如何融合【需要动态估计时核心】

**职责与输入输出。**输入几何测量或其他传感数据、时间戳、历史与运动模型，输出位置、速度等状态估计和协方差。它不是单纯平滑曲线，还要估计未直接测量的量并表达不确定性。

**课程与范围。**宾大《Robotics: Estimation and Learning》作为条件主课程：Week 1 Gaussian Model Learning，从一维、多元高斯到混合模型；Week 2 Bayesian Estimation — Target Tracking，动态系统对分布的影响与线性 Kalman filter；Week 3 Mapping，占据栅格和3D地图；Week 4 Bayesian Estimation — Localization，结合距离测量与里程计，并延伸3D定位。[26] [27]

先把 Week 1–2 连通；地图与定位属于完整体系，可继续学但不是桌面滤波实验前置。目录只核实到探索 non-linear filtering systems，不能据此虚构系统讲完 EKF、UKF。作业线索来自非官方学习者仓库，未运行；课程稳定获取、MATLAB数据和工具条件仍待实际确认。[28]

MathWorks《Understanding Kalman Filters》1–4集补滤波直觉，作者是 **Melda Ulusoy，不是 Brian Douglas**：1解释为什么估计，2讲状态观测器，3讲概率融合，4连到预测/更新。第5集只是非线性估计概念，6–7是工具演示，不替代数学课程。[29] Brunton 的 Full-State Estimation / Kalman Filter 可作控制侧观测器桥梁；Bonn 的 Bayes/Kalman/EKF 选段用于后续难点。[30] [31] [22]

**原理粒度。**理解条件概率、高斯、协方差、状态/观测模型、可观测性以及 Q/R；从预测分布与测量融合推到线性 KF 的均值、增益与协方差更新，说明模型和噪声假设。非线性模型需要时再系统补 EKF 的线性化等方法，不把短视频看完等同于掌握。

**最初实践验收。**模拟二维位置速度状态 `x=[px,py,vx,vy]ᵀ`，测量只有位置。由 `p(k+1)=p(k)+v(k)Δt` 推 A，由位置选择推 H，实现 predict/update与 Q/R。比较原始测量和估计相对真值的误差；依次增大噪声、加入加速度、漏帧，解释误差与滞后。能说明预测协方差为何增长、观测怎样约束它，以及错误时间戳怎样污染速度，才不只是“画得更平滑”。再把6A的位置测量接进同一实验。

**依赖与深入。**概率、矩阵、离散模型和5A即可，不需先学完整动力学或 RL。Cornell A5 可作相关估计练习；继续学习异常值、非线性滤波和多传感器融合，地图/定位需求再接后两周。[32]

## 7. 仿真、数据链路与系统集成【仿真核心；实机按项目】

**职责与输入输出。**保证模型、观测、动作和时间按契约连接。输入模型/驱动、控制量和传感数据，输出下一步反馈、对齐数据与可回放日志，帮助区分跨层错误。

**课程与范围。**MuJoCo Python与 Simulation 文档配 Gymnasium Env API：MJCF 的 body/joint/geom/actuator，`MjModel/MjData`、`qpos/qvel/ctrl`、`timestep/mj_step`；Gymnasium 的 `reset/step`、`terminated/truncated`、state/pixels观测。[33] [34] [35] 实机链路参考 LeRobot 标定、遥操、采集、回放、训练与 rollout；多进程驱动或跨机通信确有需要时补 ROS2 的 node、topic/service/action、launch、QoS与诊断。[36] [37]

**原理粒度。**能说明一个控制周期对应多少仿真步、actuator 如何解释 `ctrl`，以及观测/action 的 shape、范围、坐标系和单位。分清任务终止与时间截断对 RL bootstrap 的影响；记录延迟、丢帧和时间戳，而非只确认函数返回成功。

**最初实践验收。**从两关节小 MJCF 封装环境，固定 seed和初值，先跑随机策略与控制基线，再接状态策略，保存轨迹和配置。验收可回放、单位正确、观测动作对齐、误差与失败可解释；pixels另测遮挡、标定误差和延迟。统一库、源码、数据和模型版本，避免最新 main 混用旧教程。LeRobot 0.7.0 / Python ≥3.12 仅是2026-11-25已核对的版本提示，不是本日最新声明或整套兼容保证。[38]

**SO101 候选平台的实践边界。**它尚不是已购设备。五个臂关节加夹爪执行器，不是可任意控制六维末端位姿的六轴臂。已核对的标准 LeRobot 驱动采用 position模式，读 `Present_Position`、写 `Goal_Position`；torque enable/disable、`Max_Torque_Limit` 等不是校准物理力矩接口。可做模块1、2、4与位置伺服响应/跟踪，不承诺真机计算力矩、重力补偿或力控。[39] [40]

直接编程基础所需链路是 follower、电脑/USB控制板、匹配电源与固定夹具，不必先有 leader或相机；leader是官方主从遥操方案所需，视觉学习再加相机，不先强制 GPU。实验须正确标定零位，核实角度/归一化单位，独立测量末端误差，并固定底座、低速限位、设置安全停止。官方 SO101教程、驱动与 ECE4560 Joint Space 作业可作可选实践入口；后者有读取位置和时间插值目标的练习，但本版未运行作业，不视作开箱即用包。[39] [40] [41]

**依赖与深入。**Python、运动学与基本控制，连接示范、BC和RL。官方 SO101 MuJoCo 环境也是后续选项，不等于已解决真机迁移。[42] 深入接触、域随机化与 sim-to-real。CPU可做几何、KF、低维小模型和基础控制；大型视觉策略训练不保证CPU足够，渲染、作业依赖和具体机器兼容性需首次运行验证。[43]

## 8. 模仿学习：BC、分布偏移与 DAgger【路线核心】

**职责与输入输出。**从专家行为学习决策。训练输入示范观测/状态、目标和动作标签；执行输入当前信息，输出明确语义的动作，成功与否回到闭环测试。

**课程与范围。**Sanjiban Choudhury《Imitation Learning: A Series of Deep Dives》先第1–5讲：定义、BC、闭环分布偏移、DAgger与干预；Cornell A1 FetchReach 用于 BC 和研究生部分 DAgger实践。它是独立通关练习，不要求把所有学习固定在该环境。[44] [45]

**原理粒度。**推连续动作 MSE或高斯负对数似然，在明确假设下解释误差随时域累积。监督误差小不保证闭环成功；学习者会访问示范没覆盖的状态。DAgger是在学习者访问状态上请专家重标注，不是简单追加专家轨迹。

**最初实践验收。**比较专家、随机策略、BC与DAgger，统一初值、目标、动作与控制频率；比较到达即停和持续执行。报告离线loss、闭环成功率、轨迹与失败状态分布，找出loss和表现不一致的案例，检查聚合数据是否覆盖原失败区域。

**依赖与深入。**5B和基本环境接口足够开始，IK/控制器提供基线，**不必先学完 RL**。完整学习继续到第6–10讲的IRL、贝叶斯与分布匹配；ACT、Diffusion 是后续策略表达，离线 RL 是固定数据上的奖励/价值优化，都不是有示范就必须学的同义词。

## 9. 强化学习：赵世钰数学主线＋Hugging Face 补充【路线核心理论；任务算法按需】

**职责与输入输出。**在奖励目标下优化序列决策。输入状态/观测、动作、奖励、下一步信息与终止标记，学习价值或策略并产生动作；奖励不自动等于独立成功判据。

**课程与范围。**赵世钰《强化学习的数学原理》官方教材、PPT与环境代码，视频走作者B站频道。[4] [46] 章节为：1基本概念，2状态价值与Bellman方程，3最优价值与Bellman最优方程，4值迭代/策略迭代，5蒙特卡洛，6随机近似，7时序差分，8价值函数方法，9策略梯度，10Actor–Critic。已学到MC阶段，可先收尾第5章，再把第6–10章连通。

**Hugging Face 实践补充。**加入 [Deep RL Course](https://huggingface.co/learn/deep-rl-course/unit0/introduction)：免费、开源、自定进度，以图解、小测和 Google Colab 实践连接概念与训练流程。它与 Hugging Face Robotics Course 是不同课程。赵世钰承担系统数学推导，HF 补直观解释与代码实践；按正在学习的主题穿插，不要求同时从头重学两套。[48]

- **基础与表格方法：Unit 1–2。**Unit 1 回顾 RL 框架；Unit 2 对比 MC 与 TD，并讲 Q-learning。你目前学到 MC，可先读相关对比，进入 TD/Q-learning 时配合 Unit 2 实践，在 FrozenLake、Taxi 中用 Python/NumPy 自行实现表格更新，不需要神经网络。[49] [50]
- **函数近似：Unit 3 DQN。**与价值函数近似部分配合，理解 Q 表如何换成网络。Atari 实践使用 RL Baselines3 Zoo/Stable Baselines3，适合学习训练、评估和回放流程，但不等于从零实现 DQN；仍要能解释 replay、target network 和 TD loss。[49] [51]
- **策略与连续控制：Unit 4、6、8。**Unit 4 的 PyTorch 策略梯度配合赵世钰第9章；Unit 6 的 Actor-Critic/A2C 与机器人仿真实践配合第10章；之后用 Unit 8 补 PPO。机器人仿真可作为机械臂 RL 的练习入口，但不是 SO101 真机部署教程。Unity、多智能体和附加游戏按兴趣选学，不设为当前路线的前置。[49]

**使用条件。**官方已标注课程处于低维护状态，Unit 7 的 AI vs AI 功能和排行榜不再运行。理论与练习内容仍可访问，但旧 Notebook 的依赖可能需要调整，不以榜单或证书流程作为学习验收。表格练习可从 CPU 起步；Atari DQN 实践采用 GPU 加速，不承诺无独显电脑能轻松完成整套训练。[48] [50] [51]

**原理粒度。**推 MC、TD(0)、SARSA、Expected SARSA、n-step与Q-learning，分清采样来源、on/off-policy和bootstrap；理解函数近似、DQN replay/target network。由 log-derivative trick推REINFORCE，解释critic、advantage与off-policy修正。网络输出Q值和直接输出动作的接口不能混写。

**最初实践验收。**在官方grid-world自行实现MC control、TD/SARSA和Q-learning，固定种子比较回报、价值误差与样本量。仓库提供环境，不提供全部算法实现。先验收表格方法，再用小MLP替代表格Q，观察函数近似与不稳定性；从公式写算法并解释失败，比一次高回报更重要。

HF 的补充验收是：能指出 Notebook 中环境交互、动作选择和参数更新分别在哪里；修改探索率、学习率或奖励后，解释回报与策略怎样变化。使用训练库时，也要能找到关键算法实现，保存配置、模型与独立评估结果，而不只完成一次 `learn()` 调用。

**依赖与深入。**表格阶段需要概率、微积分、线代与5A，深度RL再用5B，机械臂连续控制接模块3、7。基础后用Stanford CS224R深入深度RL、离线RL、model-based与机器人专题。[47] RL不是所有机器人任务必选；控制器或BC足够时保留基线，有明确奖励优化与探索需求再引入。

## 用同一个任务贯穿：从真值到学习策略

以“机械臂末端到达目标并稳定跟踪”为贯穿任务，固定模型、关节范围、初值和目标集，包含可达、不可达、近奇异与未见目标，不把任务一开始扩大成完整抓取系统。

**真值到目标。**模块1–2在已知目标与模型下验证坐标、FK和IK，保留位置/姿态误差与解的有效性。随后模块3–4加入带时间参考，模块7记录仿真响应，得到可解释的稳定跟踪基线。这阶段要区分几何可行与动态可执行。

**视觉测量与KF。**先用仿真投影、样例图或现有图像，模块6A以手工点验证几何，再加入检测；与真值比较，隔离感知误差。目标运动或观测噪声确有需要时接6B，同时报告原测量、估计和控制响应，不把滤波误差混成控制误差。

**模仿与按需RL。**用基线采集专家轨迹，模块5、8做state-BC和DAgger，维持相同动作接口、频率和测试条件。先分析数据覆盖、标定、时序和控制误差；有明确收益再接模块9，并始终与专家、控制基线和BC比较。学得更深入与保持可诊断基线并不冲突。

**共同验收记录。**保存版本、配置、seed、初值、目标、坐标系、单位、动作定义、数据划分与原始日志。报告末端位置误差、姿态误差（任务要求时）、跟踪RMSE/最大误差、超调、延迟、控制输入或频率；视觉报告重投影与独立点误差，估计报告真值误差和协方差变化，学习报告loss、回报及闭环失败分类。成功率要附测试次数；成功阈值、维持时间和超时条件由任务预先定义，本版不编造阈值、实际成绩或运行结果。

本版列出学习与验收要求，不代表这些课程、作业或真机实验已完成运行。尚未知的是所选 Frank Park 播放列表的实际覆盖、宾大课程/作业的稳定获取与MATLAB工具条件，以及本地机器和具体版本的兼容性；这些条件在首次使用对应材料时确认即可。

## References

[1]: https://hades.mech.northwestern.edu/index.php/modern_Robotics "Modern Robotics：教材、带解练习与首尔大学历年题入口"
[2]: https://modernrobotics.northwestern.edu/nu-gm-book-resource/ "Modern Robotics：逐章配套资源"
[3]: https://www.zhihu.com/question/2085291263574786629/answer/2106704330715939686 "用户提供的机器人技术闭环文章：本版沿用既有记录"
[4]: https://github.com/MathFoundationRL/Book-Mathematical-Foundation-of-Reinforcement-Learning "赵世钰：强化学习的数学原理教材、目录与环境代码"
[5]: https://github.com/mees-robot-learning-course/ethz-course-2026/tree/main/hw2_robot_control_mdps "ETH Robot Learning 2026：HW2控制与MDP"
[6]: https://github.com/kuanfang/cornell-cs4756-2026sp/tree/master/assignments/A0 "Cornell CS4756 2026：A0 NumPy与PyTorch"
[7]: https://www.3blue1brown.com/topics/neural-networks "3Blue1Brown：神经网络系列"
[8]: https://www.3blue1brown.com/lessons/backpropagation-calculus "3Blue1Brown：Backpropagation calculus"
[9]: https://www.deeplearning.ai/specializations/deep-learning "吴恩达：Deep Learning Specialization"
[10]: https://www.deeplearning.ai/specializations/pytorch-for-deep-learning-professional-certificate "DeepLearning.AI：PyTorch for Deep Learning课程与讲师"
[11]: https://www.coursera.org/learn/pytorch-fundamentals "PyTorch Fundamentals：课程大纲"
[12]: https://docs.pytorch.org/tutorials/beginner/basics/intro.html "PyTorch：Learn the Basics"
[13]: https://www.coursera.org/learn/robotics-perception "宾大Robotics: Perception官方课程入口"
[14]: https://www.classcentral.com/course/robotics-perception-5033 "Class Central：宾大Perception目录镜像与状态记录"
[15]: https://github.com/cgebbe/coursera_robotics_perception "宾大Perception非官方学习者项目仓库"
[16]: https://fpcv.cs.columbia.edu/Monographs "Nayar：First Principles of Computer Vision官方讲义目录"
[17]: https://cave.cs.columbia.edu/Statics/monographs/Image%20Formation%20FPCV-1-1.pdf "Nayar：Image Formation FPCV-1-1"
[18]: https://cave.cs.columbia.edu/Statics/monographs/Camera%20Calibration%20FPCV-4-1.pdf "Nayar：Camera Calibration FPCV-4-1"
[19]: https://cave.cs.columbia.edu/Statics/monographs/Uncalibrated%20Stereo%20FPCV-4-2.pdf "Nayar：Uncalibrated Stereo FPCV-4-2"
[20]: https://cave.cs.columbia.edu/Statics/monographs/Optical%20Flow%20FPCV-4-3.pdf "Nayar：Optical Flow FPCV-4-3"
[21]: https://cave.cs.columbia.edu/Statics/monographs/Structure%20from%20Motion%20FPCV-4-4.pdf "Nayar：Structure from Motion FPCV-4-4"
[22]: https://www.ipb.uni-bonn.de/photo12-2021/index.html "Bonn：Photogrammetry I与II官方课程目录"
[23]: https://www.roboticsbook.org/S53_diffdrive_sensing.html "Robotics Book：5.3相机与机器人视觉"
[24]: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html "OpenCV：Camera Calibration"
[25]: https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html "OpenCV：Perspective-n-Point Pose Computation"
[26]: https://www.coursera.org/learn/robotics-estimation-learning "宾大Robotics: Estimation and Learning官方课程入口"
[27]: https://www.mooc-list.com/course/robotics-estimation-and-learning-coursera "MOOC List：宾大Estimation and Learning四周目录镜像"
[28]: https://github.com/pgeedh/RoboticsSpecialization-UPenn-Estimation-and-Learning/blob/main/README.md "宾大Estimation and Learning非官方学习者作业记录"
[29]: https://www.mathworks.com/videos/series/understanding-kalman-filters.html "MathWorks：Melda Ulusoy的Understanding Kalman Filters"
[30]: https://www.youtube.com/watch?v=MZJMi-6_4UU "Steve Brunton：Full-State Estimation"
[31]: https://www.youtube.com/watch?v=s_9InuQAx-g "Steve Brunton：The Kalman Filter"
[32]: https://github.com/kuanfang/cornell-cs4756-2026sp/blob/master/assignments/A5/A5.ipynb "Cornell CS4756 2026：A5状态估计与策略练习"
[33]: https://mujoco.readthedocs.io/en/stable/python.html "MuJoCo：Python文档"
[34]: https://mujoco.readthedocs.io/en/stable/programming/simulation.html "MuJoCo：Simulation文档"
[35]: https://gymnasium.farama.org/api/env/ "Gymnasium：Env API"
[36]: https://huggingface.co/docs/lerobot/en/il_robots "LeRobot：真实机器人模仿学习链路"
[37]: https://docs.ros.org/en/jazzy/Concepts/Basic.html "ROS2：基本概念"
[38]: https://github.com/huggingface/lerobot "LeRobot：版本、源码与数据格式入口"
[39]: https://huggingface.co/docs/lerobot/en/so101 "LeRobot：SO101官方教程"
[40]: https://raw.githubusercontent.com/huggingface/lerobot/main/src/lerobot/robots/so_follower/so_follower.py "LeRobot：SOFollower驱动源码，沿用2026-11-25核对"
[41]: https://maegantucker.com/ECE4560/assignment3-so101/ "ECE4560：SO101 Joint Space可选作业"
[42]: https://huggingface.co/lerobot/so101-tabletop-mujoco "LeRobot：SO101 Tabletop MuJoCo环境"
[43]: https://huggingface.co/docs/lerobot/en/hardware_guide "LeRobot：硬件与训练资源指南"
[44]: https://www.youtube.com/playlist?list=PLQZQ7N26C6ba2BDFVULmmBYC80cX6pNjZ "Sanjiban Choudhury：Imitation Learning: A Series of Deep Dives"
[45]: https://github.com/kuanfang/cornell-cs4756-2026sp/blob/master/assignments/A1/programming/CS4756_A1_Programming_Release.ipynb "Cornell CS4756 2026：A1 FetchReach模仿学习"
[46]: https://space.bilibili.com/2044042934 "赵世钰：作者B站频道"
[47]: https://cs224r.stanford.edu/ "Stanford CS224R：深度强化学习与机器人专题"
[48]: https://huggingface.co/learn/deep-rl-course/unit0/introduction "Hugging Face Deep RL Course：官方介绍、实践工具与维护状态"
[49]: https://raw.githubusercontent.com/huggingface/deep-rl-class/main/units/en/_toctree.yml "Hugging Face Deep RL Course：官方完整章节目录"
[50]: https://huggingface.co/learn/deep-rl-course/unit2/hands-on "Hugging Face Unit 2：从零实现FrozenLake与Taxi表格Q-learning"
[51]: https://huggingface.co/learn/deep-rl-course/en/unit3/hands-on "Hugging Face Unit 3：使用RL Baselines3 Zoo训练Atari DQN"


