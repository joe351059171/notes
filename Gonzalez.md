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
				L = 2^k
* 存储图像所需的比特数b为:
				b = M*N*k
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