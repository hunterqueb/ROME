CMD = uint8(0)
ID = uint8(2)
unencoded = [CMD ID]
encoded = cobs(unencoded)

%%
fwrite(a,encoded)
fwrite(a,uint8(0))
