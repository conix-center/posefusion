
#To select camer and gpu id to run openpose
./build/posefusion/posefusion-client2 2 --num_gpu 1 --camera 2

Capture lambda mqtt data
mosquitto_sub -h oz.andrew.cmu.edu -t /lambda/2/pose > log_grg_l2 & mosquitto_sub -h oz.andrew.cmu.edu -t /lambda/1/pose > log_grg_l1

Replay lambda mqtt data 
python3 ../python/replay2camera.py log_grg
