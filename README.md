Build3D
===

双目视觉三维重构
---

* 算法库
   * Halcon11
* 开发环境
   * VS2010 MFC
* 使用说明
   * 双目三维重构系统项目说明.docx
* 使用注意
   * 重构的图像纹理要多并清晰，尽量保持暗光，防止过曝
* 开发界面
   *    
   ![](https://github.com/HQlin/Build3D/blob/master/pic/halcon标定.png "halcon标定")
   *    
   ![](https://github.com/HQlin/Build3D/blob/master/pic/重构截图.png "重构截图")
   
---
* 算法库
   * OpenCV2.4.9
* 开发环境
   * VS2010 MFC
* 参考资料
   * https://blog.csdn.net/wangyaninglm/article/details/52142217
* 使用说明
   * stereo_calib.cpp：官方标定实例
   * stereo_match.cpp：官方三维重构实例
   * stereo_match_test.cpp：加工三维重构实例
   * 运行时候只运行一个main函数，其余两个main另命名
* 开发界面
   *
   ![](https://github.com/HQlin/Build3D/blob/master/pic/oepncv249.png "oepncv249")
   