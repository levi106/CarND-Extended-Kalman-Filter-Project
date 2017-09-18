#include "WSServer.h"

int main()
{
  ExtendedKF::WSServer server;
  server.Init();
  server.Run();
}
