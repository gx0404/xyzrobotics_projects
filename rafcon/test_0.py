box_row_num = {1:2,2:6,3:2,4:6,5:6,6:2,7:2,8:2,9:6}
pallet_tote_data = {}
for key,item in box_row_num.items():
   for i in range(item):
      box_id = str(9*i+key)
      pallet_tote_data[box_id] = {"barcode":box_id}
print(pallet_tote_data)   
import json
with open('pallet_tote_data.json', 'w') as f:
    json.dump(pallet_tote_data, f,indent=4)
   
