# WCS Adaptor

WCS Adapter is a api collection for Rafcon and WCS.
There is several files:
- rafcon.py:    rafcon api
- wcs.py:       WCS api
- zh_msg.py:    Chinese error definition
- app.py:       load all apis in rafcon and wcs. Define some global variable in rafcon and wcs.py

WCS developers and System engineer Developers can revise the implementation in wcs and rafcon.py in different projects, 
and need to follow the format to give stardand output. In rafcon.py, it is better not to add new api.
In wcs.py. It defines standard apis for customer's wcs. If necessary, we need to revise it to satisfy their need.

Feel free to add error definition in zh_msgs if necessary.