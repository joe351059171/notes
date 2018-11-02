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
### Enhancement Using Arithmetic/Logic Operations
* Image Averaging:K 增加,像素的噪声减小;当噪声图像的数量增加,平均过后的图像就越接近原始图像