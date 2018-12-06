

struct WebSvcInitMsg
{
    uint8_t Source;
    uint8_t DeviceType;
};

struct WebSvcNotifMsg
{
    uint8_t Source;
    uint8_t NotifType;
    uint8_t Status;
};

struct WebSvcCmdMsg
{
    uint8_t Source;
    uint8_t Cmd[107];
};

struct WebSvcDataMsg
{
    uint8_t Source;
    uint8_t Data[107];
};

enum {
  AM_WEBSVCINITMSG  = 100,
  AM_WEBSVCNOTIFMSG = 101,
  AM_WEBSVCCMDMSG   = 102,
  AM_WEBSVCDATAMSG  = 103,
};
