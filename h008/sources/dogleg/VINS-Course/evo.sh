export PATH=/home/ubuntu/anaconda3/bin:$PATH
#rm -f pose_output.txt
cat bin/pose_output_method3.txt | tr -s [:space:] > build/pose_output_m1_new.txt
cat bin/pose_output_LM.txt | tr -s [:space:] > build/pose_output_ori_lambda_new.txt

evo_rpe tum build/gt.txt build/pose_output_ori_lambda_new.txt -va --plot --plot_mode xz --save_results build/results/ori_lambda.zip
evo_rpe tum build/gt.txt build/pose_output_m1_new.txt -va --plot --plot_mode xz --save_results build/results/our_method.zip
evo_res build/results/*.zip -p --save_table build/results/table.csv

