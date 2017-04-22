module Main
( main
) where

import KRPCHS
import KRPCHS.SpaceCenter

import Control.Monad.Trans


main :: IO ()
main =
    withRPCClient "List biomes" "127.0.0.1" "50000" $ \client ->
        runRPCProg client listBiomes


listBiomes :: RPCContext ()
listBiomes =
    getActiveVessel               >>= \vessel ->
    getVesselOrbit         vessel >>= \orbit  ->
    getOrbitBody           orbit  >>= \body   ->
    getCelestialBodyBiomes body   >>= \biomes ->
        liftIO $ print biomes
