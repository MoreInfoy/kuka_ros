#!/usr/bin/expect
set timeout 2000
set password "nesa"
set i 0
while {$i < 1} {
	spawn scp /home/nimpng/kuka_ros/src/sendmsg.tar.gz nesa320@10.15.198.151:/home/nesa320/yangshunpeng/ggcnn-master/
	expect "nesa320@10.15.198.151"
	send "$password\n"
	expect "sendmsg.xml                                     100%"
	spawn scp nesa320@10.15.198.151:/home/nesa320/yangshunpeng/ggcnn-master/result.npy /home/nimpng/kuka_ros/src/
	expect "nesa320@10.15.198.151"
	send "$password\n"
	expect "result.npz                                      100%"
	incr i
	# puts "loops : $i"
}


	

