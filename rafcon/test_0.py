box_row_num = {1:2,2:2,3:2,4:2,5:2}
pallet_tote_data = {}
for key,item in box_row_num.items():
   for i in range(item):
      box_id = str(5*i+key)
      pallet_tote_data[box_id] = {"barcode":box_id}
print(pallet_tote_data)   
import json
with open('pallet_tote_data.json', 'w') as f:
    json.dump(pallet_tote_data, f,indent=4)
   
