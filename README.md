# SAPMTK311.c

[ZC-SAPMTK311](http://www.dgzcled.com/products/WH4530A.html)是一颗国产的接近传感器，采用I2C接口，有环境光（ALS）和接近（PS）两个功能。

- SAPMTK311.c 是单颗ZC-SAPMTK311的I2C接口的驱动程序
- SAPMTK311_Example.c 是单颗ZC-SAPMTK311的I2C接口的驱动程序的示例文件，你可以在这里面写一些测试代码
- SAPMTK311_Array.c 是多颗ZC-SAPMTK311阵列的封装，适用于一颗MCU控制多颗ZC-SAPMTK311

# 如何使用

整个驱动程序基于I2C接口，所以你需要有一个I2C读写的接口函数，根据你的硬件平台和框架，修改`SAPMTK311.c`中的`I2C_Write`和`I2C_Read`函数，然后就可以使用这个驱动程序了。

参数里的`i2c_id`用来区别不同的传感器（因为I2C地址是相同的），`device_addr`是传感器地址，`reg`是寄存器地址，`data`是数据，`len`是数据长度。

# 注意事项

驱动程序中没有对I2C的初始化、读写进行错误处理，所以你需要在调用驱动程序之前，确保I2C正常工作。