          

char* gGetBinVersion(char* full_data, int full_data_len, char* substr)
{
    if (full_data == NULL || full_data_len <= 0 || substr == NULL) {
        return NULL;
    }
 
    if (*substr == '\0') {
        return NULL;
    }
    int sublen = strlen(substr);
    int i;
    char* cur = full_data;
    int last_possible = full_data_len - sublen + 1;
    for (i = 0; i < last_possible; i++) {
        if (*cur == *substr) {
            //assert(full_data_len - i >= sublen);
            if (memcmp(cur, substr, sublen) == 0) {
                //found
                return cur-3;
            }
        }
        cur++;
    }
    return NULL;
}
void dfu_demo(void)
{
    char *version; 
    unsigned char versiontag[]= {0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0};

    version = gGetBinVersion(gFileBuf, FileLength, (char* )versiontag);
    if(version)
    {
        printk("version is %02x %02x %02x %02x\n",version[0],version[1],version[2],version[3]);
        printk("version is %02x %02x %02x %02x\n",version[4],version[5],version[6],version[7]);
    }
}