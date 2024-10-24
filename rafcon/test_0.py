box_row_num = {1:4,2:4,3:4,4:4,5:4,6:4,7:4,8:4,9:4}
pallet_tote_data = {}
for key,item in box_row_num.items():
   for i in range(item):
      box_id = str(9*i+key)
      pallet_tote_data[box_id] = {"barcode":box_id}
print(pallet_tote_data)   
import json
with open('pallet_tote_data.json', 'w') as f:
    json.dump(pallet_tote_data, f,indent=4)
   
