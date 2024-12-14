WGS84ToUTM 
-- 用于将经纬度转换为墨卡托投影，并且与雷达消息的时间戳进行对齐（采用线性差值的方式。）
-- 输入：raw_gnss.csv   pointcloud.csv_time_vector2.csv.csv
-- 输出：RTK.csv



WGS84ToUTMForCheckRTKPercision 
-- 用于将经纬度转换为墨卡托投影，没有与雷达消息进行对齐，方便直接查看RTK信息是否可靠。查看是否存在尖刺。
-- 输入：raw_gnss.csv
-- 输出：RTK.csv

pointcloud.csv_time_vector2.csv.csv得到方式：将bag包里面的/raw_gnss话题转换为raw_gnss.csv文件
rostopic echo -b front line_no10.bag -p /raw_gnss >raw_gnss.csv

pointcloud.csv_time_vector2.csv.csv得到方式：as you like
只要能得到Radar数据的时间戳就可以了
