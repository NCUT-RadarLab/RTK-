WGS84ToUTM 
-- 用于将经纬度转换为墨卡托投影，并且与雷达消息的时间戳进行对齐（采用线性差值的方式。）
-- 输入：raw_gnss.csv   pointcloud.csv_time_vector2.csv.csv
-- 输出：RTK.csv



WGS84ToUTMForCheckRTKPercision 
-- 用于将经纬度转换为墨卡托投影，没有与雷达消息进行对齐，方便直接查看RTK信息是否可靠。查看是否存在尖刺。
-- 输入：raw_gnss.csv
-- 输出：RTK.csv
