﻿# remote-locker-device

Modifiez les lignes suivantes au début de **src/main.cpp** pour adapter le programme à un backend particulier :

```cpp
// Server details
const char server[]   =         "admin.eristich.dev";
const char resource[] =         "/webhook";
const int  port       =         443;
const char webhook_api_key[] =  "<API_KEY>";
```
