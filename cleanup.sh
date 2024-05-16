#!/bin/bash
 
# Set the path of the folder to be cleaned
folder_path1="/home/xyz/xyz_log/wopt_log/"
folder_path2="/home/xyz/xyz_log/mf_log"
folder_path3="/home/xyz/xyz_log/smart_planner_log"
 
# Set the maximum age (in days) for files to be deleted
max_age=3
 
# Function to perform cleanup
perform_cleanup() {
    find "$folder_path1" -type f -mtime +$max_age -delete
    find "$folder_path1" -type f -mtime $max_age -delete
    find "$folder_path1" -type d -empty -delete
    
    find "$folder_path2" -type f -mtime +$max_age -delete
    find "$folder_path2" -type f -mtime $max_age -delete
    find "$folder_path2" -type d -empty -delete
 
    find "$folder_path3" -type f -mtime +$max_age -delete
    find "$folder_path3" -type f -mtime $max_age -delete
    find "$folder_path3" -type d -empty -delete
    
    echo "Cleanup completed!"
}
 
# Infinite loop for continuous monitoring
while true; do
    # Perform cleanup
    perform_cleanup
 
    # Sleep for 24 hours
    sleep 24h
done
