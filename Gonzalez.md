## 2  Digital Image Fundamentals
### 2.1  Elements of Visual Perception
**一些人眼的构成,人眼成像的细节,错觉**

![人眼](/Users/chika/Desktop/notes/人眼.png)
* 实验数据指出，主观亮度（即由人的视觉系统感觉到的亮度）是进入眼睛的光强度的对数函数。
* 韦伯率（ΔIc/I）实验表明，在低的照明级别，亮度辨别较差（韦伯比大），当背景照明增加时亮度辨别得到改善（韦伯比降低）。
* 感觉亮度不是简单的强度函数（马赫带效应、同时对比度效应）。
### 2.2  Light and the Electromagnetic Spectrum
* 电磁波谱的可见波段大约占据430nm（紫色）~790nm（红色）的范围。
###2.3  Image Sensing and Acquisition
* 传感器的三种排列方式,点,线,面
* ,用f(x,y),二维函数形式表示图像，可由两个分量来表征：（1）入射到观察场的光源总量和；（2）场景中物体反射光的总量 
			f(x,y)=i(x,y)r(x,y)

上式中反射分量限制在0（全吸收）和1（全反射）之间。i(x,y)
的性质取决于照射源，而r(x,y)取决于成像物体的特性。
***
### 2.4  Image Sampling and Quantization
* 坐标(coordinate)数字化叫抽样(sampling);幅度(amplitude)数字化叫量化(quantization)
* 抽样后的排列
图像f(x,y)可以表示成矩阵形式
![矩阵表示](/Users/chika/Desktop/notes/图像的矩阵表示1.png)
或更传统的形式
![矩阵表示](/Users/chika/Desktop/notes/图像的矩阵表示2.png)
* 处于处理存储和抽样硬件考虑,灰度的总数目灰度级常常是2的幂次:
				L = 2<sup>k</sup>
* 存储图像所需的比特数b为:
				b = M&times;N&times;k
这是很好理解的:M行N列的图像有MxN个像素,每个像素有k种取值.
* 当一幅图像有2^k个灰度,就称其为一幅k比特图像或k位图像.比如256个灰度级的图像就是8比特图像
* 空间分辨率:一幅图中最小的可分辨细节
* false contouring:insufficient number of gray levels in smooth areas of a digital image causes a ridgelike structures.通常在16或更少灰度级的图像中比较明显.
* isopreference curve:同时改变N和k造成的影响.
![iso-pic](/Users/chika/Desktop/notes/iso-pic.png)
![iso-curve](/Users/chika/Desktop/notes/iso-curve.png)
发现:等偏爱曲线倾向右上方,这很好理解,N和K越大,图像细节越好,质量越高.
值得注意的是,**有大量细节的图像只需要很少的灰度级**,以及对于lena和摄影师,在一定程度上,尽管图像质量保持相同(很短的一段),但k实际上是在下降的.可能的原因时k的减小导致了图的对比度明显增加,人眼把它当作了图片质量的改善.
* 减少抽样图像混叠的方法是通过模糊处理减少高频分量.尽管无论如何混叠都是存在的.混叠频率的影响可以在合适条件下以Moiré Pattern的形式被观测到.
![摩尔条纹](/Users/chika/Desktop/notes/moire_pattern.png)
* nearest neighbor interpolation:考虑500x500的图像要放大到750x750.创建一个750x750的网格,与原始图像有相同间隔,然后收缩,使其与原图匹配.显然收缩后像素间隔要小于500x500,我们就在原图中找最近的像素,把值赋给新的像素点.完成之后扩展到正常的750x750.
* bilinear interpolation:用四个最近邻去估计给定位置的灰度.
				v(x,y) = ax+by+cxy+d
### 2.5 Some Basic Relationships Between Pixels
* pixel(x,y) has 4-neighbors:(x+1,y),(x-1,y),(x,y+1),(x,y-1).This set is denoted as N<sub>4</sub>(p).and 4 diagonal neighbors:(x+1,y+1),(x+1,y-1),(x-1,y+1),(x-1,y-1),denoted as N<sub>D</sub>(P).And the 8 neighbors are denoted by N<sub>8</sub>(P).

* 两个像素联通的概念:they are neighbors and if their gary levels satisfy a specified criterion of similarity.

* Adjacency:

  1. 4-adjacency:p&q have values from V and q is N<sub>4</sub>(p).
  2. 8-adjacency:q is N<sub>8</sub>(p).
  3. m-adjacency: a. q is in N<sub>4</sub>(P) or N<sub>D</sub>(P)  b. N<sub>4</sub>(P) $\bigcap$ N<sub>D</sub>(P) has no pixels whose values are from V.

  混合临接是为了消除8-adjacency的ambiguity.

* Path(curve):像素p(x<sub>0</sub>,y<sub>0</sub>)到像素q(s,t)有一系列的点,这些点是两两相邻的.如果首尾相等,则曲线封闭.由像素的邻接方式分4-path,8-path,m-path.
* Connected set:p和q的通路组成了S中所有像素.对任一p,连通到p的像素集是一个连通分量.如果S中只有一个连通分量,则S是连通集.
* Region:一个连通集又叫一个区域.
* Boundary(Border/Contour):有一个或以上的neighbors不在R里的像素的集合

## 3  Image Enhancement in the Spatial Domain
* 主旨是处理图像让它更适合特定应用处理
* 没有固定评判标准
### 3.1 Background
* 单点操作:gray-level(also called intensity or mapping) transformation funciton
* larger neighbors:masks(also referred to as filters,kernels,templates,or windows) 主要方法就是用(x,y)点的neighbor的值去确定(x,y)的值
### 3.2 Some Basic Gray Level Transformation
* 反转intensity level特别适合处理在黑色区域的白或灰色细节(mammogram)
* log用来增加图片中的黑色像素的取值,压缩更高程度的值.(傅立叶频谱图)
* Power-Law transformation s = cr<sup>&gamma;</sup> ,取决于&gamma;的值,十分灵活;Gamma Correction(CRT);P-L trans也对提高对比度很有用.
* Piecewise-Linear Transformation:Contrast Stretching;分段的两个点满足一定条件就是thresholding:gray-level slicing:强调某个范围的灰度,要么对range of interest内highlight,其他的调低,要么调高ROI,其他不变;bit-plane slicing,highlight the contribution made to total image appearance by specific bits,获取主要成分可以thresholding
### 3.3 Histogram Processing
低对比度集中在一坨,高对比度就覆盖了很宽的范围,像素分布也近似均匀分布
* Histogram Equalization,要求变换函数T(r)单值且在区间内单增,且0$\le$T(r)$\le$1 for 0 $\le$ r $\le$ 1
* T(r)和p<sub>r</sub>(r)有关,但结果上的p<sub>s</sub>(s)总是均匀的,独立于p<sub>r</sub>(r)的.p是概率密度函数
* histogram equalization: s<sub>k</sub> = $\sum_{j=0}^k$ $\frac{n_j}n$  k = 0,1,2,……,L-1  使柱状图更扩展到整个图使灰度范围变大,加强对比度
* histogram matching(specification):有些情况均匀地增强并不是最好的.我们由输入得p,进而得s,从而求得变换函数G(z),反解出z = G<sup>-1</sup>(s),这个从s到z的映射就是我们期望的.一般步骤就是先均衡得s,对均衡图像中值为s的每个像素进行反映射得相应像素值z,处理完毕后就得到输出图像的PDF等于指定的PDF.
* 局部增强:neighbor,一个一个移,计算邻域中的点的直方图,但通常会产生checkerboard effect.
* 直方图的一些数据:均值、方差.用来局部增强.

![局部增强](/Users/chika/Desktop/notes/局部增强.png)

* 通过全局均值和局部均值找出暗区域,通过方差(对比度)确定暗区域中需要增强的点
### 3.4 Enhancement Using Arithmetic/Logic Operations
* Image Averaging:K 增加,像素的噪声减小;当噪声图像的数量增加,平均过后的图像就越接近原始图像
### 3.5 Basics of Spatial Filtering
* filter,mask,kernel,template,window实际上都是一张subimage,它的值被叫做系数而不是像素
* 为了区别频域,我们把这种处理叫做spatial fitering
* R = $\sum_{i=1}^9$$\omega_i$$\Zeta_i$  3&times;3的常用公式
* 对于filter到了边界的情况,我们要么规定filter中心不能超过(n-1)/2,即filter不能越界,这种情况我们得到了一个比原图片小的用full mask处理过的图片;要么我们不管越界部分的filter,只用部分的mask去处理图片;或者把原图扩展,在边缘加0(或其他值),处理完之后再消掉这些padding
### 3.6 Smoothing Spatial Filters
* smoothing filters are used for blurring and for noise reduction
* *averaging filters(lowpass filters)* :**linear**,give the average of the pixels contained in the neighborhood of the filter mask as a response.
* smoothing filter可以降噪是因为噪声主要存在于灰度的一个突变中,通过把一个像素换为它邻近区域的一个均值,可以减小突变.然而edge也会被减小,这是不希望的
* weighted average : pixels are multiplied by different coefficients,thus giving more improtance to some pixels at the expense of others.Basic strategy is to reduce blurring in the smoothing process.
* order-statstics filters:**nonlinear**,其中最广为人知的代表就是median filter,把像素的值换成整个区域的中间值.对于去除椒盐噪声极其有效
### 3.7 Sharpening Spatial Filters
* 对于一阶微分的定义一般要保证:1. 在恒定灰度区域微分值不变. 2. 在灰度台阶或斜坡处微分值为0. 3. 沿斜坡微分值为0. 

* 类似的,二阶微分要保证:1. 在恒定区域微分值为0. 2. 在灰度台阶或斜坡的起点处微分值非零. 3. 沿斜坡微分值非零

* 一阶差分公式:	$\frac{\partial f}{\partial x}$ = f(x+1) - f(x)

* 二阶差分公式:	$\frac{\partial^2 f}{\partial x^2}$ = f(x+1) + f(x-1) -2f(x)

* 边缘在灰度上来说常常类似ramp,而由图可以看出

  ![一阶&二阶差分](/Users/chika/Desktop/notes/一阶和二阶差分.png)

在ramp的部分一阶差分一直都是非零的,也就是说如果用一阶差分处理过后的图像边缘部分一般是很宽的;而二阶微分在ramp的起始位置是从负到0到正的,这就意味着会产生一个双边缘

* Laplacian: 各向同性	$\nabla^2f$ = $\frac{\partial^2f}{\partial x^2}$ + $\frac{\partial^2f}{\partial y^2}$	将其离散,有

  $\frac{\partial^2f}{\partial x^2}$ = f(x+1,y) + f(x-1,y) - 2f(x,y)     [x方向];

  同理有 

  $\frac{\partial^2f}{\partial y^2}$ = f(x,y+1) + f(x,y-1) - 2f(x,y)	   [y方向];

  那么两个变量的离散拉普拉斯算子就是

  $\nabla^2f(x,y)$ = $f(x+1,y)+f(x-1,y)+f(x,y+1)+f(x,y-1)-4f(x,y)$

  它的滤波模版如图3.7(a);至于对角线方向,我们在对角线方向加入两项,然后在中新元素减去即可;实际中也有用如c或d这样的滤波模版,差别仅仅是符号而已,在用拉普拉斯算子滤波过后的图像与其他图像合并是考虑到符号差异来进行加减即可,即中心系数为正时用+号,为负用-号

  ![laplacian滤波模版](/Users/chika/Desktop/notes/laplacian滤波模版.png)

需要注意的是,在实际中,也可以略过先滤波再加减图像的步骤,可以直接一步到位获得锐化图像.即直接加上一个f(x,y)就可以,即3.37中的c,d的中心系数从4变为了5,从8变为了9(或a,b减小到-5,-9)

+ unsharp masking : 1,模糊原图;2,从原图中减去模糊图像(剩下来的部分称为模版);						1.$f_s(x,y)=f(x,y)-\bar f(x,y)$ 

* high-boost filtering:             2.$f_{hb}(x,y)=Af(x,y)-\bar f(x,y)$ 

  high-boost filtering is a slight further generalization of unsharp masking;A$\geq$1,化简后有$f_{hb}(x,y)=(A-1)f(x,y)+f_s(x,y)$	,$f_s$代表锐化过后的图像,实际上它并不要求必须是unsharp masking得到的,比如我们用laplacian,那$f_{hb}$就变成了$Af(x,y)\pm\nabla^2f(x,y)$;实际上当A=1的时候,high-boost过滤就是个标准的Laplacian sharpening,A超过1且越来越大,那锐化的作用也在逐渐减小,如果A足够大,high-boost处理过后的图片大概就和原图乘了个常数没啥区别

* Gradient: $\nabla f$ &equiv;$grad(f)$&equiv;$$\begin{bmatrix}G_x\\ G_y\end{bmatrix}$$&equiv;$$\begin{bmatrix} \frac{\partial f}{\partial x}\\ \frac{\partial f}{\partial y}\end{bmatrix}$$,就像我们学过的,梯度指出了点(x,y)处f的最大变化率的方向;而它的magnitude表示为$M(x,y)=mag(\nabla f)=\sqrt(G^2_x+G^2_y)$,需要注意M是和原图像大小相同的图像,常称梯度图像;为了减小运算负担,我们常用绝对值来近似$M(x,y)\approx\left|G_x\right|+\left|G_y\right|$

* Robert corss-gradient operators&Sobel operators,梯度可消除慢变化背景,增强小突变

  ![交叉梯度算子和索贝尔算子](/Users/chika/Desktop/notes/交叉梯度算子&索贝尔算子.png)
## 4  Image Enhancement in the Frequency Domain
### 4.2 Introduction to the Fourier Transform and the Frequency Domain
* 傅立叶变换对
    $F(u,v) = $$$\int_{-\infty}^\infty\int_{-\infty}^\infty{f(x,y)e^{-j2\pi(ux+vy)}\,{\rm dx\rm dy}}$$

    $f(x,y) = $$$\int_{-\infty}^\infty\int_{-\infty}^\infty{F(u,v)e^{j2\pi(ux+vy)}\,{\rm du\rm dv}}$$

* 我们更关心离散的情况,因此:

    $F(u) = \frac{1}{M}\sum_{x=0}^{M-1}f(x)e^{-j2{\pi}ux/M}$	$for\ u = 0,1,2,...,M-1.$

    $f(x) = \sum^{M-1}_{u=0}F(u)e^{j2{\pi}ux/M}$	$for\ x = 0,1,2,...,M-1.$
 对于DFT和IDFT,1/M放在哪里都可以,或者两个前头都乘1/$\sqrt M$

 又有欧拉公式:$e^{j\theta} = cos\theta + jsin\theta$ ,离散傅立叶变换F(u)可化为$F(u) = \frac{1}{M}\sum\limits^{M-1}_{x=0}f(x)[cos2{\pi}ux/M - jsin2{\pi}ux/M]$
 可化为极坐标系$F(u) = \left|F(u)\right|e^{-j\phi(u)}$,其中$\left|F(u)\right| = [R^2(u)+I^2(u)]^{1/2}$,$\phi(u) = tan^{-1}[\frac{I(u)}{R(u)}]$,功率谱:$P(u) = \left|F(u)\right|^2=R^2(u)+I^2(u)$

* 空间域X轴和曲线围成的面积加倍,则频域主频的高度翻倍;X轴翻倍,零点个数翻倍

* $\Delta u = \frac{1}{M\Delta x}$,这是空间域和频率域的关系

* 以上是一维时的情况,接下来二维:

  $F(u,v)=\frac{1}{MN}\sum\limits^{M-1}_{x=0}\sum\limits^{N-1}_{y=0}f(x,y)e^{-j2\pi(ux/M+vy/N)}$,u = 0,1,2,...,M-1;v = 0,1,2,...,N-1;

  $f(x,y)=\sum\limits^{M-1}_{u=0}\sum\limits^{N-1}_{v=0}F(u,v)e^{j2\pi{ux/M+vy/N}}$,x = 0,1,2,...,M-1;y = 0,1,2,...,N-1

* 由于指数的一些性质,有$\frak F$$[f(x,y)(-1)^{x+y}]=F(u-M/2,v-N/2)$,$\frak F[·]$ denotes Fourier transform.这也就是说,给f(x,y)乘上$(-1)^{x+y}$,就会把F(u,v)给一到左边(M/2,N/2)的位置,即M&times;N的中心,我们把M&times;N这块区域叫做frequency rectangle

* 有个性质,$F(0,0)=\frac{1}{MN}\sum^{M-1}_{x=0}\sum^{N-1}_{y=0}f(x,y)$,即是说,傅立叶变换的原点的值,就是整张图灰度值的平均

* 二维有这个关系:$\Delta u=\frac{1}{M\Delta x}$;$\Delta v= \frac{1}{N\Delta y}$

* 在频率域过滤是很简单的,它一般由一下组成:
  1. 给输入图像乘个$(-1)^{x+y}$
  2. 计算1的DFT得到F(u,v)
  3. 给F(u,v)乘上过滤函数H(u,v)
  4. 计算3的IDFT
  5. 取4的实部
  6. 给5乘个$(-1)^{x+y}$

* notch filter:由于要使总的灰度为0,因此图像总体会变黑;边缘凸显

  $H(u,v)=\begin{cases} 0,&\text{if (u,v)=(M/2,N/2)}\\1,&\text{otherwise.}\end{cases}$

* 低通blur;高通sharp,with little smooth gray-level detail

* 在空间域和频率域过滤的对应关系:convolution theorem

  $f(x,y)*h(x,y)=\frac{1}{MN}\sum\limits^{M-1}_{m=0}\sum\limits^{N-1}_{n=0}f(m,n)h(x-m,y-n)$

  $f(x,y)*h(x,y)\iff F(u,v)H(u,v)$&&$f(x,y)h(x,y)\iff F(u,v)*H(u,v)$

  we need one more concept before completing the tie between the spatial and frequency domains.An impulse function of strength A,located at coordinates$(x_0,y_0)$,is denoted by $A\delta(x-x_0,y-y_0)$and is defined by the expression:

  ​	$\sum\limits^{M-1}_{x=0}\sum\limits^{N-1}_{y=0}s(x,y)A\delta(x-x_0,y-y_0)=As(x_0,y_0)$

  这就是说s(x,y)乘冲激函数的和就是冲激位置的那个函数值乘上冲激函数的幅值A

* sifting property of the impulse function:一个函数和冲激函数的卷积就是复制在冲激位置的函数值;当冲激函数位于原点时:

  $\sum\limits^{M-1}_{x=0}\sum\limits^{N-1}_{y=0}s(x,y)\delta(x,y)=s(0,0)$

* 有了上述概念,我们就可以引出最有趣和有用的关系(其实就是证出了对于一个filter,它的DFT和IDFT构成了一个傅立叶变换对)

  $\frak F$$[\delta(x,y)]=\frac{1}{MN}\sum\limits^{M-1}_{x=0}\sum\limits^{N-1}_{y=0}\delta(x,y)e^{-j2\pi(ux/M+vy/N)}=\frac{1}{MN}$

  $\delta(x,y)*h(x,y)=\frac{1}{MN}\sum\limits^{M-1}_{m=0}\sum\limits^{N-1}_{n=0}\delta(m,n)h(x-m,y-n)=\frac{1}{MN}h(x,y)$

  我们知道:$f(x,y)*h(x,y)\iff F(u,v)H(u,v)$,那把f(x,y)换成$\delta(x,y)$,就是$\delta(x,y)*h(x,y)\iff$$\frak F$$[\delta(x,y)]H(u,v)$,又$\frak F$$[\delta(x,y)]$是个常数,左边的结果也是个常数乘h(x,y),那么就有$h(x,y)\iff H(u,v)$

* 我们常用基于高斯函数的过滤因为高斯函数的形状简单,而且他的变换都是高斯实函数:$H(u)=Ae^{-u^2/2\sigma^2}$,它的逆变换$h(x)=\sqrt{2\pi}Ae^{-2\pi^2\sigma^2x^2}$,可以看到他们两随$\sigma$的变换,形状刚好是呈相反趋势变化的
![高斯滤波](/Users/chika/Desktop/notes/高斯滤波.png)

### 4.3 Smoothing Frequecy-Domain Filters
* 主要三种低通滤波器:ideal,Butterworth&Gaussian filters;ideal very sharp ,Guassian very smooth,Butterworth is a transition between 2 extremes,order越高越接近理想,越低越接近高斯

* ILPF:$H(u,v)=\begin{cases} 1,&\text{if D(u,v) $\leq D_0$}\\0,&\text{if D(u,v) $> D_0$}\end{cases}$,$D_0$是指定的非负值,D(u,v)是点(u,v)到频率矩形原点的距离.

  ![ILPF](/Users/chika/Desktop/notes/ILPF.png)

  圆内无衰减,圆外完全被衰减

* $P_T=\sum\limits^{M-1}_{u=0}\sum\limits^{N-1}_{v=0}P(u,v)$,计算图像总的功率,然后通过$\alpha=100[\sum\limits_{u}\sum\limits_vP(u,v)/P_T]$,求得当前的subimage占总的功率的多少,然后这个对应了功率图中圆占的面积.![功率圆](/Users/chika/Desktop/notes/功率圆.png)

  随着$\alpha$的增大,高频的增加使细节越来越好;“ringing”是ILPF的的一种特性

* ![ringing的解释](/Users/chika/Desktop/notes/ringing的解释.png)

* BLPF:$H(u,v)=\frac{1}{1+[D(u,v)/D_0]^{2n}}$,Ringing generally is imperceptible in filters of order 2, 一阶完全没有,高阶就会越来越明显;
* GLPF:$H(u,v)=e^{-D^2(u,v)/{2\sigma^2}}$通过令$\sigma=D_0$,有$H(u,v)=e^{-D^2(u,v)/{2D_0^2}}$,$D_0$是截止频率,当$D(u,v)=D_0$时,GLPF下降到最大值的0.607.GLPF平滑的效果比二阶的BLPF要差一点,但他没有ringing,这是很重要的
### 4.4 Sharpening Frequency Domain Filters
* $H_{hp}(u,v)=1-H_{lp}(u,v)$低通通过的频率,高通截止,低通不通的频率高通通过

* IHPF:$H(u,v)=\begin{cases} 0,&\text{if D(u,v) $\leq D_0$}\\1,&\text{if D(u,v) $> D_0$}\end{cases}$,可以想见的是IHPF也是有ringing的

  ![HPF剖面图](/Users/chika/Desktop/notes/HPF剖面图.png)

* BHPF:$H(u,v)=\frac{1}{1+[D_0/D(u,v)]^{2n}}$

* GHPF:$H(u,v)=1-e^{-D^2(u,v)/{2D_0^2}}$,高斯滤波得到的结果更加平滑,效果较好

* ![IHPF滤波](/Users/chika/Desktop/notes/IHPF滤波.png)

  ​	![BHPF滤波](/Users/chika/Desktop/notes/BHPF滤波.png)

  ![GHPF滤波](/Users/chika/Desktop/notes/GHPF滤波.png)

* Laplacian in the Frequency Domain