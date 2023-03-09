
import os

# input_file = "../Results/kf_V101-monotoa_est.txt"
# output_file = "../results_tum_format/kf_v101_mono"
# input_file = "../Results_baseline/kf_V101_monoeuroc.txt" 
# output_file = "../results_tum_format/kf_v101_mono_base"

# def convert_files (input_file, output_file):
#     with open(input_file, 'r') as f_in, open(output_file, 'w') as f_out:
#        for line in f_in:
#            if line.startswith('#'):
#                # Skip any commented lines
#                continue
           
#            # Split the line into timestamp and pose components
#            timestamp, *pose = line.strip().split()
           
#            # Convert the timestamp from nanoseconds to seconds
#            timestamp_sec = float(timestamp) / 1e9
           
#            # Write the updated line to the output file
#            f_out.write("{:.6f} {}\n".format(timestamp_sec, ' '.join(pose)))



def convert_folders(input_folder, output_folder):
    # create the output folder if it doesn't exist
    os.makedirs(output_folder, exist_ok=True)
    
    # loop through all files in the input folder
    for filename in os.listdir(input_folder):
        input_path = os.path.join(input_folder, filename)
        output_path = os.path.join(output_folder, filename[:-4] + "_tum.txt")

        if not os.path.exists(output_path):
            # create the output file if it doesn't exist
            with open(output_path, 'w'):
                pass
        
        with open(input_path, 'r') as f_in, open(output_path, 'w') as f_out:
            for line in f_in:
                if line.startswith('#'):
                    # Skip any commented lines
                    continue
                
                # Split the line into timestamp and pose components
                timestamp, *pose = line.strip().split()
                
                # Convert the timestamp from nanoseconds to seconds
                timestamp_sec = float(timestamp) / 1e9
                
                # Write the updated line to the output file
                f_out.write("{:.6f} {}\n".format(timestamp_sec, ' '.join(pose)))
            
    print("Conversion complete.")

# example usage
input_folder1 = "/home/meisam/ORB_SLAM3/Results"
input_folder2 = "/home/meisam/ORB_SLAM3/Results_baseline/euroc_mono"
input_folder2 = "/home/meisam/ORB_SLAM3/Results_baseline/euroc_inertial"
folders = (input_folder1, input_folder2)
for in_folder in folders:
    output_folder = "/home/meisam/ORB_SLAM3/results_tum_format/inertial"
    convert_folders(in_folder, output_folder)


