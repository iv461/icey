class Promise:
    def __init__(self, fn=None):
        self.value = None
        self.handlers = []
        if fn is not None: fn(self.fulfill, self.reject)

    def fulfill(self, value):
        self.value = (value, None)
        for cb in self.handlers: cb()

    def reject(self, err):
        self.value = (None, err)
        for cb in self.handlers: cb()

    def register(self, handler): self.handlers.append(handler)
  
    def then(self, onFulfilled, onRejected=lambda x: (None, x)):
        def handler_(resolve, reject):
            def cb():
                val, err = self.value
                if val is not None:
                    if onFulfilled is None:
                        return
                    res = onFulfilled(val)
                else:
                    if onRejected is None:
                        return
                    res = onRejected(err)
                if res is None: # user Callback returned none
                    return
                if isinstance(res, tuple): # If callback returns regular value, it is intepreted as no error
                    vf, ef = res
                else: # It has to return a tuple so that it can propagate the error
                    vf = res
                if vf is not None:
                    resolve(vf)
                else:
                    reject(ef)
            return self.register(cb)
        return Promise(handler_)

    def catch(self, onRejected): return self.then(onFulfilled=None, onRejected=onRejected)

def make_promise():
    return Promise()

def marker(i, behavior):
    def log_i(val):
        print(f"marker{i} with val: {val}")
        if behavior == "Some":
            return f"marker{i}"
        elif behavior == "None":
            return None 
        elif behavior == "Err":
            return None, f"error{i}"
    return log_i

my_promise1 = make_promise()
my_promise1\
    .then(marker(1, "Some"))\
    .then(marker(2, "Some"))\
    .then(marker(3, "Some"))\
    .catch(marker(4, "Some"))\
    .then(marker(5, "Some"))

print("ChainTest")
my_promise1.fulfill("hello")

print("ExceptionFallthroughTest")
my_promise1.reject("BiGExcEption")

print("VoidCatchTest")
my_promise2 = make_promise()
my_promise2\
    .then(marker(1, "Some"))\
    .then(marker(2, "Err"))\
    .then(marker(3, "Some"))\
    .catch(marker(4, "None"))\
    .then(marker(5, "Some"))

my_promise2.reject("BiGExcEption2")

print("ThenErroringTest")
my_promise3 = make_promise()
my_promise3\
    .then(marker(1, "Err"))\
    .then(marker(2, "Err"))\
    .then(marker(3, "Some"))\
    .catch(marker(4, "Err"))\
    .catch(marker(5, "Err"))\
    .then(marker(6, "Some"))\
    .then(marker(7, "Err"))\

my_promise3.fulfill("GoodVal")
    

