box_row_num = {1:6,2:6,3:6,4:5,5:5,6:5,7:5,8:5,9:5}
pallet_tote_data = {}
for key,item in box_row_num.items():
   for i in range(item):
      box_id = str(9*i+key)
      pallet_tote_data[box_id] = {"barcode":box_id}
print(pallet_tote_data)   
import json
with open('pallet_tote_data.json', 'w') as f:
    json.dump(pallet_tote_data, f,indent=4)
   
