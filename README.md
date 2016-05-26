# KRPC Hors Service

KRPC Hors Service is a client for [KRPC](https://github.com/krpc/krpc) -- a mod for Kerbal Space Program -- written in Haskell.


## Example of usage

### Simple RPC

```haskell
import KRPCHS

main :: IO ()
main =
    -- Let's start by creating an RPCClient
    withRPCClient "Demo" "127.0.0.1" "50000" $ \client -> do
        -- From here, you can run programs that live in an RPCContext
        runRPCProg client liftOffProgram
        runRPCProg client munTransferProgram
        runRPCProg client munLandingProgram

liftOffProgram :: RPCContext ()
liftOffProgram = do
    ut <- getUT           -- get the date
    v  <- getActiveVessel -- get our current vessel
    liftIO $ putStrLn $ "Mission start at " ++ show ut
    ... do some other things

munTransferProgram :: RPCContext ()
munTransferProgram = do
    ...

munLandingProgram :: RPCContext ()
munLandingProgram = do
    ...
```

Under the hood, an RPCContext is a ReaderT on top of IO.


### Using streams

```haskell
import KRPCHS

main :: IO ()
main =
    withRPCClient    "Demo" "127.0.0.1" "50000" $ \client -> do
    withStreamClient client "127.0.0.1" "50001" $ \streamClient -> do
        utStream <- runRPCProg client makeUTStream
        runStreamProg streamClient (streamProg utStream)

makeUTStream :: RPCContext (KRPCStream Double)
makeUTStream = getUTStream

streamProg :: KRPCStream Double -> StreamContext ()
streamProg utStream = forever $ do
    msg <- getStreamMessage
    ut  <- getStreamResult utStream msg
    liftIO $ putStrLn (show ut)
```

Similar to the RPCContext, a StreamContext is a ReaderT on top of IO.


You might have noticed that all KRPC functions (such as `getUT`, `getUTStream`) are tiny programs you can call directly using `runRPCProg`.


## TODO

* Documentation
* Cleanup the code generator and make it available
* Find a better way to handle exceptions
