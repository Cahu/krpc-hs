**This is no longer actively maintained.**

# KRPC Hors-Service

KRPC Hors-Service is a client for [kRPC] -- a mod for Kerbal Space Program --
written in Haskell. Please refer to the [kRPC documentation] for a detailed
list of available RPC calls.

[kRPC]: https://github.com/krpc/krpc
[kRPC documentation]: http://krpc.github.io/krpc/index.html

## How to use

*   The simplest way to use this library is with
	[Stack](http://docs.haskellstack.org) by adding the dependence to your
	project's stack.yaml file (replace vX.Y.Z by the release tag you want to use or
	a commit hash):

	```yaml
	# Local packages, usually specified by relative directory name
	packages:
	- '.'
	- extra-dep: true
	  location:
	    git: git@github.com:cahu/krpc-hs
	    commit: vX.Y.Z
	```

*   Alternatively, you can clone the project in a directory of your choice and run
	these commands:
	``` bash
	~/krpc-hs$ stack build
	```
	You can use haddock to generate basic documentation, for instance:
	```bash
	~/krpc-hs$ stack haddock
	```
	Tell stack where to find krpc-hs in your project's `stack.yaml` file:
	```yaml
	# Local packages, usually specified by relative directory name
	packages:
	- '.'
	- '/home/user/krpc-hs'
	```

*   If you don't want to use stack, then the usual `cabal build` should work
	with maybe some small tweaking of the `.cabal` file.


## Examples

Complete working examples can be found in the `examples` folder.

### Simple RPC

```haskell
import KRPCHS
import KRPCHS.UI
import KRPCHS.SpaceCenter

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
	countDown 10          -- call another program
    ut <- getUT           -- get the date
    v  <- getActiveVessel -- get our current vessel
    ... do some other things

munTransferProgram :: RPCContext ()
munTransferProgram = do
    ...

munLandingProgram :: RPCContext ()
munLandingProgram = do
    ...

countDown :: Int -> RPCContext ()
countDown n = forM_ (reverse [ 1 .. n ]) $ \sec -> do
    void   $ message (T.pack $ show sec) 1 MessagePosition'TopCenter
    liftIO $ threadDelay 1000000
```


### Using streams

```haskell
import KRPCHS
import KRPCHS.SpaceCenter

main :: IO ()
main =
    withRPCClient    "Demo" "127.0.0.1" "50000" $ \client ->
    withStreamClient client "127.0.0.1" "50001" $ \streamClient ->
        runRPCProg client (exampleProg streamClient)

exampleProg :: StreamClient -> RPCContext ()
exampleProg streamClient = do
    utStream <- getUTStream
    forever $ do
        msg <- getStreamMessage streamClient
        when (messageHasResultFor utStream msg) $ do
            ut <- getStreamResult utStream msg
            liftIO $ putStrLn (show ut)
```


## Notes

* `RPCContext` is a ReaderT on top of IO and it derives MonadThrow, MonadCatch,
  MonadMask (from the `exception` module).
* You should always check that a stream message has a result for the stream you
  are going to use with `messageHasResultFor` because there will be some delay
  between the moment you ask for a new stream and the moment you start
  receiving results. In the event you use `getStreamResult` on a message with
  no result for the specified stream, an exception will be thrown.
