## Retargeting printf
See section 16.5.6 of IDE, 

for the _Newlib_ (and NewlibNano) 
To retarget printf we need to write our version of _write: `int _write(int iFileHandle, char *pcBuffer, int iLength)` the function returns number of unwritten bytes if error, otherwise 0 for success. More information as also explained in [Red Hat newlib]

We couldn't retargeting 

Change to Redlib (nohost)
int __sys_write(int iFileHandle, char *pcBuffer, int iLength);
int __sys_readc(void);

[Red Hat newlib] https://sourceware.org/newlib/libc.html#Syscalls



