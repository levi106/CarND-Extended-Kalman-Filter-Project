#ifndef WSSERVER_H_
#define WSSERVER_H_

#include <memory>
#include <experimental/propagate_const>

namespace ExtendedKF {
class WSServer {
public:
  /**
  * Constructor.
  */
  WSServer();

  /**
  * Destructor.
  */
  virtual ~WSServer();

  /**
  * Initialize Server
  */
  void Init();

  /**
  * Start Server
  */
  void Run();

private:
  class impl;
  std::experimental::propagate_const<std::unique_ptr<impl>> pImpl;
};
}

#endif /* WSSERVER_H_ */
