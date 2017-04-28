module KRPCHS.Drawing
( Line
, Polygon
, Text
, addDirection
, addDirectionStream
, addDirectionStreamReq
, addLine
, addLineStream
, addLineStreamReq
, addPolygon
, addPolygonStream
, addPolygonStreamReq
, addText
, addTextStream
, addTextStreamReq
, clear
, lineRemove
, getLineColor
, getLineColorStream
, getLineColorStreamReq
, getLineEnd
, getLineEndStream
, getLineEndStreamReq
, getLineMaterial
, getLineMaterialStream
, getLineMaterialStreamReq
, getLineReferenceFrame
, getLineReferenceFrameStream
, getLineReferenceFrameStreamReq
, getLineStart
, getLineStartStream
, getLineStartStreamReq
, getLineThickness
, getLineThicknessStream
, getLineThicknessStreamReq
, getLineVisible
, getLineVisibleStream
, getLineVisibleStreamReq
, setLineColor
, setLineEnd
, setLineMaterial
, setLineReferenceFrame
, setLineStart
, setLineThickness
, setLineVisible
, polygonRemove
, getPolygonColor
, getPolygonColorStream
, getPolygonColorStreamReq
, getPolygonMaterial
, getPolygonMaterialStream
, getPolygonMaterialStreamReq
, getPolygonReferenceFrame
, getPolygonReferenceFrameStream
, getPolygonReferenceFrameStreamReq
, getPolygonThickness
, getPolygonThicknessStream
, getPolygonThicknessStreamReq
, getPolygonVertices
, getPolygonVerticesStream
, getPolygonVerticesStreamReq
, getPolygonVisible
, getPolygonVisibleStream
, getPolygonVisibleStreamReq
, setPolygonColor
, setPolygonMaterial
, setPolygonReferenceFrame
, setPolygonThickness
, setPolygonVertices
, setPolygonVisible
, textRemove
, getTextAlignment
, getTextAlignmentStream
, getTextAlignmentStreamReq
, getTextAnchor
, getTextAnchorStream
, getTextAnchorStreamReq
, getTextAvailableFonts
, getTextAvailableFontsStream
, getTextAvailableFontsStreamReq
, getTextCharacterSize
, getTextCharacterSizeStream
, getTextCharacterSizeStreamReq
, getTextColor
, getTextColorStream
, getTextColorStreamReq
, getTextContent
, getTextContentStream
, getTextContentStreamReq
, getTextFont
, getTextFontStream
, getTextFontStreamReq
, getTextLineSpacing
, getTextLineSpacingStream
, getTextLineSpacingStreamReq
, getTextMaterial
, getTextMaterialStream
, getTextMaterialStreamReq
, getTextPosition
, getTextPositionStream
, getTextPositionStreamReq
, getTextReferenceFrame
, getTextReferenceFrameStream
, getTextReferenceFrameStreamReq
, getTextRotation
, getTextRotationStream
, getTextRotationStreamReq
, getTextSize
, getTextSizeStream
, getTextSizeStreamReq
, getTextStyle
, getTextStyleStream
, getTextStyleStreamReq
, getTextVisible
, getTextVisibleStream
, getTextVisibleStreamReq
, setTextAlignment
, setTextAnchor
, setTextCharacterSize
, setTextColor
, setTextContent
, setTextFont
, setTextLineSpacing
, setTextMaterial
, setTextPosition
, setTextReferenceFrame
, setTextRotation
, setTextSize
, setTextStyle
, setTextVisible
) where

import qualified Data.Int
import qualified Data.Text
import qualified KRPCHS.SpaceCenter
import qualified KRPCHS.UI

import Control.Monad.Catch
import Control.Monad.IO.Class

import KRPCHS.Internal.Requests
import KRPCHS.Internal.SerializeUtils


{-
 - A line. Created using <see cref="M:Drawing.AddLine" />.
 -}
newtype Line = Line { lineId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Line where
    encodePb   = encodePb . lineId
    decodePb b = Line <$> decodePb b

instance KRPCResponseExtractable Line

{-
 - A polygon. Created using <see cref="M:Drawing.AddPolygon" />.
 -}
newtype Polygon = Polygon { polygonId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Polygon where
    encodePb   = encodePb . polygonId
    decodePb b = Polygon <$> decodePb b

instance KRPCResponseExtractable Polygon

{-
 - Text. Created using <see cref="M:Drawing.AddText" />.
 -}
newtype Text = Text { textId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Text where
    encodePb   = encodePb . textId
    decodePb b = Text <$> decodePb b

instance KRPCResponseExtractable Text



{-
 - Draw a direction vector in the scene, from the center of mass of the active vessel.<param name="direction">Direction to draw the line in.<param name="referenceFrame">Reference frame that the direction is in.<param name="length">The length of the line.<param name="visible">Whether the line is visible.
 -}
addDirection :: (MonadIO m, MonadThrow m, MonadRPC m) => (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> Float -> Bool -> m (KRPCHS.Drawing.Line)
addDirection directionArg referenceFrameArg lengthArg visibleArg = do
    let r = makeRequest "Drawing" "AddDirection" [makeArgument 0 directionArg, makeArgument 1 referenceFrameArg, makeArgument 2 lengthArg, makeArgument 3 visibleArg]
    res <- sendRequest r
    processResponse res

addDirectionStreamReq :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> Float -> Bool -> KRPCStreamReq (KRPCHS.Drawing.Line)
addDirectionStreamReq directionArg referenceFrameArg lengthArg visibleArg =
    let req = makeRequest "Drawing" "AddDirection" [makeArgument 0 directionArg, makeArgument 1 referenceFrameArg, makeArgument 2 lengthArg, makeArgument 3 visibleArg]
    in  makeStream req

addDirectionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> Float -> Bool -> m (KRPCStream (KRPCHS.Drawing.Line))
addDirectionStream directionArg referenceFrameArg lengthArg visibleArg = requestStream $ addDirectionStreamReq directionArg referenceFrameArg lengthArg visibleArg 

{-
 - Draw a line in the scene.<param name="start">Position of the start of the line.<param name="end">Position of the end of the line.<param name="referenceFrame">Reference frame that the positions are in.<param name="visible">Whether the line is visible.
 -}
addLine :: (MonadIO m, MonadThrow m, MonadRPC m) => (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> Bool -> m (KRPCHS.Drawing.Line)
addLine startArg endArg referenceFrameArg visibleArg = do
    let r = makeRequest "Drawing" "AddLine" [makeArgument 0 startArg, makeArgument 1 endArg, makeArgument 2 referenceFrameArg, makeArgument 3 visibleArg]
    res <- sendRequest r
    processResponse res

addLineStreamReq :: (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> Bool -> KRPCStreamReq (KRPCHS.Drawing.Line)
addLineStreamReq startArg endArg referenceFrameArg visibleArg =
    let req = makeRequest "Drawing" "AddLine" [makeArgument 0 startArg, makeArgument 1 endArg, makeArgument 2 referenceFrameArg, makeArgument 3 visibleArg]
    in  makeStream req

addLineStream :: (MonadIO m, MonadThrow m, MonadRPC m) => (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> Bool -> m (KRPCStream (KRPCHS.Drawing.Line))
addLineStream startArg endArg referenceFrameArg visibleArg = requestStream $ addLineStreamReq startArg endArg referenceFrameArg visibleArg 

{-
 - Draw a polygon in the scene, defined by a list of vertices.<param name="vertices">Vertices of the polygon.<param name="referenceFrame">Reference frame that the vertices are in.<param name="visible">Whether the polygon is visible.
 -}
addPolygon :: (MonadIO m, MonadThrow m, MonadRPC m) => [(Double, Double, Double)] -> KRPCHS.SpaceCenter.ReferenceFrame -> Bool -> m (KRPCHS.Drawing.Polygon)
addPolygon verticesArg referenceFrameArg visibleArg = do
    let r = makeRequest "Drawing" "AddPolygon" [makeArgument 0 verticesArg, makeArgument 1 referenceFrameArg, makeArgument 2 visibleArg]
    res <- sendRequest r
    processResponse res

addPolygonStreamReq :: [(Double, Double, Double)] -> KRPCHS.SpaceCenter.ReferenceFrame -> Bool -> KRPCStreamReq (KRPCHS.Drawing.Polygon)
addPolygonStreamReq verticesArg referenceFrameArg visibleArg =
    let req = makeRequest "Drawing" "AddPolygon" [makeArgument 0 verticesArg, makeArgument 1 referenceFrameArg, makeArgument 2 visibleArg]
    in  makeStream req

addPolygonStream :: (MonadIO m, MonadThrow m, MonadRPC m) => [(Double, Double, Double)] -> KRPCHS.SpaceCenter.ReferenceFrame -> Bool -> m (KRPCStream (KRPCHS.Drawing.Polygon))
addPolygonStream verticesArg referenceFrameArg visibleArg = requestStream $ addPolygonStreamReq verticesArg referenceFrameArg visibleArg 

{-
 - Draw text in the scene.<param name="text">The string to draw.<param name="referenceFrame">Reference frame that the text position is in.<param name="position">Position of the text.<param name="rotation">Rotation of the text, as a quaternion.<param name="visible">Whether the text is visible.
 -}
addText :: (MonadIO m, MonadThrow m, MonadRPC m) => Data.Text.Text -> KRPCHS.SpaceCenter.ReferenceFrame -> (Double, Double, Double) -> (Double, Double, Double, Double) -> Bool -> m (KRPCHS.Drawing.Text)
addText textArg referenceFrameArg positionArg rotationArg visibleArg = do
    let r = makeRequest "Drawing" "AddText" [makeArgument 0 textArg, makeArgument 1 referenceFrameArg, makeArgument 2 positionArg, makeArgument 3 rotationArg, makeArgument 4 visibleArg]
    res <- sendRequest r
    processResponse res

addTextStreamReq :: Data.Text.Text -> KRPCHS.SpaceCenter.ReferenceFrame -> (Double, Double, Double) -> (Double, Double, Double, Double) -> Bool -> KRPCStreamReq (KRPCHS.Drawing.Text)
addTextStreamReq textArg referenceFrameArg positionArg rotationArg visibleArg =
    let req = makeRequest "Drawing" "AddText" [makeArgument 0 textArg, makeArgument 1 referenceFrameArg, makeArgument 2 positionArg, makeArgument 3 rotationArg, makeArgument 4 visibleArg]
    in  makeStream req

addTextStream :: (MonadIO m, MonadThrow m, MonadRPC m) => Data.Text.Text -> KRPCHS.SpaceCenter.ReferenceFrame -> (Double, Double, Double) -> (Double, Double, Double, Double) -> Bool -> m (KRPCStream (KRPCHS.Drawing.Text))
addTextStream textArg referenceFrameArg positionArg rotationArg visibleArg = requestStream $ addTextStreamReq textArg referenceFrameArg positionArg rotationArg visibleArg 

{-
 - Remove all objects being drawn.<param name="clientOnly">If true, only remove objects created by the calling client.
 -}
clear :: (MonadIO m, MonadThrow m, MonadRPC m) => Bool -> m ()
clear clientOnlyArg = do
    let r = makeRequest "Drawing" "Clear" [makeArgument 0 clientOnlyArg]
    res <- sendRequest r
    processResponse res 

{-
 - Remove the object.
 -}
lineRemove :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Line -> m ()
lineRemove thisArg = do
    let r = makeRequest "Drawing" "Line_Remove" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Set the color
 -}
getLineColor :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Line -> m ((Double, Double, Double))
getLineColor thisArg = do
    let r = makeRequest "Drawing" "Line_get_Color" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLineColorStreamReq :: KRPCHS.Drawing.Line -> KRPCStreamReq ((Double, Double, Double))
getLineColorStreamReq thisArg =
    let req = makeRequest "Drawing" "Line_get_Color" [makeArgument 0 thisArg]
    in  makeStream req

getLineColorStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Line -> m (KRPCStream ((Double, Double, Double)))
getLineColorStream thisArg = requestStream $ getLineColorStreamReq thisArg 

{-
 - End position of the line.
 -}
getLineEnd :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Line -> m ((Double, Double, Double))
getLineEnd thisArg = do
    let r = makeRequest "Drawing" "Line_get_End" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLineEndStreamReq :: KRPCHS.Drawing.Line -> KRPCStreamReq ((Double, Double, Double))
getLineEndStreamReq thisArg =
    let req = makeRequest "Drawing" "Line_get_End" [makeArgument 0 thisArg]
    in  makeStream req

getLineEndStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Line -> m (KRPCStream ((Double, Double, Double)))
getLineEndStream thisArg = requestStream $ getLineEndStreamReq thisArg 

{-
 - Material used to render the object.
 - Creates the material from a shader with the given name.
 -}
getLineMaterial :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Line -> m (Data.Text.Text)
getLineMaterial thisArg = do
    let r = makeRequest "Drawing" "Line_get_Material" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLineMaterialStreamReq :: KRPCHS.Drawing.Line -> KRPCStreamReq (Data.Text.Text)
getLineMaterialStreamReq thisArg =
    let req = makeRequest "Drawing" "Line_get_Material" [makeArgument 0 thisArg]
    in  makeStream req

getLineMaterialStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Line -> m (KRPCStream (Data.Text.Text))
getLineMaterialStream thisArg = requestStream $ getLineMaterialStreamReq thisArg 

{-
 - Reference frame for the positions of the object.
 -}
getLineReferenceFrame :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Line -> m (KRPCHS.SpaceCenter.ReferenceFrame)
getLineReferenceFrame thisArg = do
    let r = makeRequest "Drawing" "Line_get_ReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLineReferenceFrameStreamReq :: KRPCHS.Drawing.Line -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getLineReferenceFrameStreamReq thisArg =
    let req = makeRequest "Drawing" "Line_get_ReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getLineReferenceFrameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Line -> m (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getLineReferenceFrameStream thisArg = requestStream $ getLineReferenceFrameStreamReq thisArg 

{-
 - Start position of the line.
 -}
getLineStart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Line -> m ((Double, Double, Double))
getLineStart thisArg = do
    let r = makeRequest "Drawing" "Line_get_Start" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLineStartStreamReq :: KRPCHS.Drawing.Line -> KRPCStreamReq ((Double, Double, Double))
getLineStartStreamReq thisArg =
    let req = makeRequest "Drawing" "Line_get_Start" [makeArgument 0 thisArg]
    in  makeStream req

getLineStartStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Line -> m (KRPCStream ((Double, Double, Double)))
getLineStartStream thisArg = requestStream $ getLineStartStreamReq thisArg 

{-
 - Set the thickness
 -}
getLineThickness :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Line -> m (Float)
getLineThickness thisArg = do
    let r = makeRequest "Drawing" "Line_get_Thickness" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLineThicknessStreamReq :: KRPCHS.Drawing.Line -> KRPCStreamReq (Float)
getLineThicknessStreamReq thisArg =
    let req = makeRequest "Drawing" "Line_get_Thickness" [makeArgument 0 thisArg]
    in  makeStream req

getLineThicknessStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Line -> m (KRPCStream (Float))
getLineThicknessStream thisArg = requestStream $ getLineThicknessStreamReq thisArg 

{-
 - Whether the object is visible.
 -}
getLineVisible :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Line -> m (Bool)
getLineVisible thisArg = do
    let r = makeRequest "Drawing" "Line_get_Visible" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLineVisibleStreamReq :: KRPCHS.Drawing.Line -> KRPCStreamReq (Bool)
getLineVisibleStreamReq thisArg =
    let req = makeRequest "Drawing" "Line_get_Visible" [makeArgument 0 thisArg]
    in  makeStream req

getLineVisibleStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Line -> m (KRPCStream (Bool))
getLineVisibleStream thisArg = requestStream $ getLineVisibleStreamReq thisArg 

{-
 - Set the color
 -}
setLineColor :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Line -> (Double, Double, Double) -> m ()
setLineColor thisArg valueArg = do
    let r = makeRequest "Drawing" "Line_set_Color" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - End position of the line.
 -}
setLineEnd :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Line -> (Double, Double, Double) -> m ()
setLineEnd thisArg valueArg = do
    let r = makeRequest "Drawing" "Line_set_End" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Material used to render the object.
 - Creates the material from a shader with the given name.
 -}
setLineMaterial :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Line -> Data.Text.Text -> m ()
setLineMaterial thisArg valueArg = do
    let r = makeRequest "Drawing" "Line_set_Material" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Reference frame for the positions of the object.
 -}
setLineReferenceFrame :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Line -> KRPCHS.SpaceCenter.ReferenceFrame -> m ()
setLineReferenceFrame thisArg valueArg = do
    let r = makeRequest "Drawing" "Line_set_ReferenceFrame" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Start position of the line.
 -}
setLineStart :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Line -> (Double, Double, Double) -> m ()
setLineStart thisArg valueArg = do
    let r = makeRequest "Drawing" "Line_set_Start" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Set the thickness
 -}
setLineThickness :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Line -> Float -> m ()
setLineThickness thisArg valueArg = do
    let r = makeRequest "Drawing" "Line_set_Thickness" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the object is visible.
 -}
setLineVisible :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Line -> Bool -> m ()
setLineVisible thisArg valueArg = do
    let r = makeRequest "Drawing" "Line_set_Visible" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Remove the object.
 -}
polygonRemove :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Polygon -> m ()
polygonRemove thisArg = do
    let r = makeRequest "Drawing" "Polygon_Remove" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Set the color
 -}
getPolygonColor :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Polygon -> m ((Double, Double, Double))
getPolygonColor thisArg = do
    let r = makeRequest "Drawing" "Polygon_get_Color" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPolygonColorStreamReq :: KRPCHS.Drawing.Polygon -> KRPCStreamReq ((Double, Double, Double))
getPolygonColorStreamReq thisArg =
    let req = makeRequest "Drawing" "Polygon_get_Color" [makeArgument 0 thisArg]
    in  makeStream req

getPolygonColorStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Polygon -> m (KRPCStream ((Double, Double, Double)))
getPolygonColorStream thisArg = requestStream $ getPolygonColorStreamReq thisArg 

{-
 - Material used to render the object.
 - Creates the material from a shader with the given name.
 -}
getPolygonMaterial :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Polygon -> m (Data.Text.Text)
getPolygonMaterial thisArg = do
    let r = makeRequest "Drawing" "Polygon_get_Material" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPolygonMaterialStreamReq :: KRPCHS.Drawing.Polygon -> KRPCStreamReq (Data.Text.Text)
getPolygonMaterialStreamReq thisArg =
    let req = makeRequest "Drawing" "Polygon_get_Material" [makeArgument 0 thisArg]
    in  makeStream req

getPolygonMaterialStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Polygon -> m (KRPCStream (Data.Text.Text))
getPolygonMaterialStream thisArg = requestStream $ getPolygonMaterialStreamReq thisArg 

{-
 - Reference frame for the positions of the object.
 -}
getPolygonReferenceFrame :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Polygon -> m (KRPCHS.SpaceCenter.ReferenceFrame)
getPolygonReferenceFrame thisArg = do
    let r = makeRequest "Drawing" "Polygon_get_ReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPolygonReferenceFrameStreamReq :: KRPCHS.Drawing.Polygon -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getPolygonReferenceFrameStreamReq thisArg =
    let req = makeRequest "Drawing" "Polygon_get_ReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getPolygonReferenceFrameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Polygon -> m (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getPolygonReferenceFrameStream thisArg = requestStream $ getPolygonReferenceFrameStreamReq thisArg 

{-
 - Set the thickness
 -}
getPolygonThickness :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Polygon -> m (Float)
getPolygonThickness thisArg = do
    let r = makeRequest "Drawing" "Polygon_get_Thickness" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPolygonThicknessStreamReq :: KRPCHS.Drawing.Polygon -> KRPCStreamReq (Float)
getPolygonThicknessStreamReq thisArg =
    let req = makeRequest "Drawing" "Polygon_get_Thickness" [makeArgument 0 thisArg]
    in  makeStream req

getPolygonThicknessStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Polygon -> m (KRPCStream (Float))
getPolygonThicknessStream thisArg = requestStream $ getPolygonThicknessStreamReq thisArg 

{-
 - Vertices for the polygon.
 -}
getPolygonVertices :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Polygon -> m ([(Double, Double, Double)])
getPolygonVertices thisArg = do
    let r = makeRequest "Drawing" "Polygon_get_Vertices" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPolygonVerticesStreamReq :: KRPCHS.Drawing.Polygon -> KRPCStreamReq ([(Double, Double, Double)])
getPolygonVerticesStreamReq thisArg =
    let req = makeRequest "Drawing" "Polygon_get_Vertices" [makeArgument 0 thisArg]
    in  makeStream req

getPolygonVerticesStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Polygon -> m (KRPCStream ([(Double, Double, Double)]))
getPolygonVerticesStream thisArg = requestStream $ getPolygonVerticesStreamReq thisArg 

{-
 - Whether the object is visible.
 -}
getPolygonVisible :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Polygon -> m (Bool)
getPolygonVisible thisArg = do
    let r = makeRequest "Drawing" "Polygon_get_Visible" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPolygonVisibleStreamReq :: KRPCHS.Drawing.Polygon -> KRPCStreamReq (Bool)
getPolygonVisibleStreamReq thisArg =
    let req = makeRequest "Drawing" "Polygon_get_Visible" [makeArgument 0 thisArg]
    in  makeStream req

getPolygonVisibleStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Polygon -> m (KRPCStream (Bool))
getPolygonVisibleStream thisArg = requestStream $ getPolygonVisibleStreamReq thisArg 

{-
 - Set the color
 -}
setPolygonColor :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Polygon -> (Double, Double, Double) -> m ()
setPolygonColor thisArg valueArg = do
    let r = makeRequest "Drawing" "Polygon_set_Color" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Material used to render the object.
 - Creates the material from a shader with the given name.
 -}
setPolygonMaterial :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Polygon -> Data.Text.Text -> m ()
setPolygonMaterial thisArg valueArg = do
    let r = makeRequest "Drawing" "Polygon_set_Material" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Reference frame for the positions of the object.
 -}
setPolygonReferenceFrame :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Polygon -> KRPCHS.SpaceCenter.ReferenceFrame -> m ()
setPolygonReferenceFrame thisArg valueArg = do
    let r = makeRequest "Drawing" "Polygon_set_ReferenceFrame" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Set the thickness
 -}
setPolygonThickness :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Polygon -> Float -> m ()
setPolygonThickness thisArg valueArg = do
    let r = makeRequest "Drawing" "Polygon_set_Thickness" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Vertices for the polygon.
 -}
setPolygonVertices :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Polygon -> [(Double, Double, Double)] -> m ()
setPolygonVertices thisArg valueArg = do
    let r = makeRequest "Drawing" "Polygon_set_Vertices" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the object is visible.
 -}
setPolygonVisible :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Polygon -> Bool -> m ()
setPolygonVisible thisArg valueArg = do
    let r = makeRequest "Drawing" "Polygon_set_Visible" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Remove the object.
 -}
textRemove :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m ()
textRemove thisArg = do
    let r = makeRequest "Drawing" "Text_Remove" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Alignment.
 -}
getTextAlignment :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m (KRPCHS.UI.TextAlignment)
getTextAlignment thisArg = do
    let r = makeRequest "Drawing" "Text_get_Alignment" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextAlignmentStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq (KRPCHS.UI.TextAlignment)
getTextAlignmentStreamReq thisArg =
    let req = makeRequest "Drawing" "Text_get_Alignment" [makeArgument 0 thisArg]
    in  makeStream req

getTextAlignmentStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m (KRPCStream (KRPCHS.UI.TextAlignment))
getTextAlignmentStream thisArg = requestStream $ getTextAlignmentStreamReq thisArg 

{-
 - Anchor.
 -}
getTextAnchor :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m (KRPCHS.UI.TextAnchor)
getTextAnchor thisArg = do
    let r = makeRequest "Drawing" "Text_get_Anchor" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextAnchorStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq (KRPCHS.UI.TextAnchor)
getTextAnchorStreamReq thisArg =
    let req = makeRequest "Drawing" "Text_get_Anchor" [makeArgument 0 thisArg]
    in  makeStream req

getTextAnchorStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m (KRPCStream (KRPCHS.UI.TextAnchor))
getTextAnchorStream thisArg = requestStream $ getTextAnchorStreamReq thisArg 

{-
 - A list of all available fonts.
 -}
getTextAvailableFonts :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m ([Data.Text.Text])
getTextAvailableFonts thisArg = do
    let r = makeRequest "Drawing" "Text_get_AvailableFonts" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextAvailableFontsStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq ([Data.Text.Text])
getTextAvailableFontsStreamReq thisArg =
    let req = makeRequest "Drawing" "Text_get_AvailableFonts" [makeArgument 0 thisArg]
    in  makeStream req

getTextAvailableFontsStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m (KRPCStream ([Data.Text.Text]))
getTextAvailableFontsStream thisArg = requestStream $ getTextAvailableFontsStreamReq thisArg 

{-
 - Character size.
 -}
getTextCharacterSize :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m (Float)
getTextCharacterSize thisArg = do
    let r = makeRequest "Drawing" "Text_get_CharacterSize" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextCharacterSizeStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq (Float)
getTextCharacterSizeStreamReq thisArg =
    let req = makeRequest "Drawing" "Text_get_CharacterSize" [makeArgument 0 thisArg]
    in  makeStream req

getTextCharacterSizeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m (KRPCStream (Float))
getTextCharacterSizeStream thisArg = requestStream $ getTextCharacterSizeStreamReq thisArg 

{-
 - Set the color
 -}
getTextColor :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m ((Double, Double, Double))
getTextColor thisArg = do
    let r = makeRequest "Drawing" "Text_get_Color" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextColorStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq ((Double, Double, Double))
getTextColorStreamReq thisArg =
    let req = makeRequest "Drawing" "Text_get_Color" [makeArgument 0 thisArg]
    in  makeStream req

getTextColorStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m (KRPCStream ((Double, Double, Double)))
getTextColorStream thisArg = requestStream $ getTextColorStreamReq thisArg 

{-
 - The text string
 -}
getTextContent :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m (Data.Text.Text)
getTextContent thisArg = do
    let r = makeRequest "Drawing" "Text_get_Content" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextContentStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq (Data.Text.Text)
getTextContentStreamReq thisArg =
    let req = makeRequest "Drawing" "Text_get_Content" [makeArgument 0 thisArg]
    in  makeStream req

getTextContentStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m (KRPCStream (Data.Text.Text))
getTextContentStream thisArg = requestStream $ getTextContentStreamReq thisArg 

{-
 - Name of the font
 -}
getTextFont :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m (Data.Text.Text)
getTextFont thisArg = do
    let r = makeRequest "Drawing" "Text_get_Font" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextFontStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq (Data.Text.Text)
getTextFontStreamReq thisArg =
    let req = makeRequest "Drawing" "Text_get_Font" [makeArgument 0 thisArg]
    in  makeStream req

getTextFontStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m (KRPCStream (Data.Text.Text))
getTextFontStream thisArg = requestStream $ getTextFontStreamReq thisArg 

{-
 - Line spacing.
 -}
getTextLineSpacing :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m (Float)
getTextLineSpacing thisArg = do
    let r = makeRequest "Drawing" "Text_get_LineSpacing" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextLineSpacingStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq (Float)
getTextLineSpacingStreamReq thisArg =
    let req = makeRequest "Drawing" "Text_get_LineSpacing" [makeArgument 0 thisArg]
    in  makeStream req

getTextLineSpacingStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m (KRPCStream (Float))
getTextLineSpacingStream thisArg = requestStream $ getTextLineSpacingStreamReq thisArg 

{-
 - Material used to render the object.
 - Creates the material from a shader with the given name.
 -}
getTextMaterial :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m (Data.Text.Text)
getTextMaterial thisArg = do
    let r = makeRequest "Drawing" "Text_get_Material" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextMaterialStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq (Data.Text.Text)
getTextMaterialStreamReq thisArg =
    let req = makeRequest "Drawing" "Text_get_Material" [makeArgument 0 thisArg]
    in  makeStream req

getTextMaterialStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m (KRPCStream (Data.Text.Text))
getTextMaterialStream thisArg = requestStream $ getTextMaterialStreamReq thisArg 

{-
 - Position of the text.
 -}
getTextPosition :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m ((Double, Double, Double))
getTextPosition thisArg = do
    let r = makeRequest "Drawing" "Text_get_Position" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextPositionStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq ((Double, Double, Double))
getTextPositionStreamReq thisArg =
    let req = makeRequest "Drawing" "Text_get_Position" [makeArgument 0 thisArg]
    in  makeStream req

getTextPositionStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m (KRPCStream ((Double, Double, Double)))
getTextPositionStream thisArg = requestStream $ getTextPositionStreamReq thisArg 

{-
 - Reference frame for the positions of the object.
 -}
getTextReferenceFrame :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m (KRPCHS.SpaceCenter.ReferenceFrame)
getTextReferenceFrame thisArg = do
    let r = makeRequest "Drawing" "Text_get_ReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextReferenceFrameStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getTextReferenceFrameStreamReq thisArg =
    let req = makeRequest "Drawing" "Text_get_ReferenceFrame" [makeArgument 0 thisArg]
    in  makeStream req

getTextReferenceFrameStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getTextReferenceFrameStream thisArg = requestStream $ getTextReferenceFrameStreamReq thisArg 

{-
 - Rotation of the text as a quaternion.
 -}
getTextRotation :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m ((Double, Double, Double, Double))
getTextRotation thisArg = do
    let r = makeRequest "Drawing" "Text_get_Rotation" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextRotationStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq ((Double, Double, Double, Double))
getTextRotationStreamReq thisArg =
    let req = makeRequest "Drawing" "Text_get_Rotation" [makeArgument 0 thisArg]
    in  makeStream req

getTextRotationStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m (KRPCStream ((Double, Double, Double, Double)))
getTextRotationStream thisArg = requestStream $ getTextRotationStreamReq thisArg 

{-
 - Font size.
 -}
getTextSize :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m (Data.Int.Int32)
getTextSize thisArg = do
    let r = makeRequest "Drawing" "Text_get_Size" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextSizeStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq (Data.Int.Int32)
getTextSizeStreamReq thisArg =
    let req = makeRequest "Drawing" "Text_get_Size" [makeArgument 0 thisArg]
    in  makeStream req

getTextSizeStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m (KRPCStream (Data.Int.Int32))
getTextSizeStream thisArg = requestStream $ getTextSizeStreamReq thisArg 

{-
 - Font style.
 -}
getTextStyle :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m (KRPCHS.UI.FontStyle)
getTextStyle thisArg = do
    let r = makeRequest "Drawing" "Text_get_Style" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextStyleStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq (KRPCHS.UI.FontStyle)
getTextStyleStreamReq thisArg =
    let req = makeRequest "Drawing" "Text_get_Style" [makeArgument 0 thisArg]
    in  makeStream req

getTextStyleStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m (KRPCStream (KRPCHS.UI.FontStyle))
getTextStyleStream thisArg = requestStream $ getTextStyleStreamReq thisArg 

{-
 - Whether the object is visible.
 -}
getTextVisible :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m (Bool)
getTextVisible thisArg = do
    let r = makeRequest "Drawing" "Text_get_Visible" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextVisibleStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq (Bool)
getTextVisibleStreamReq thisArg =
    let req = makeRequest "Drawing" "Text_get_Visible" [makeArgument 0 thisArg]
    in  makeStream req

getTextVisibleStream :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> m (KRPCStream (Bool))
getTextVisibleStream thisArg = requestStream $ getTextVisibleStreamReq thisArg 

{-
 - Alignment.
 -}
setTextAlignment :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> KRPCHS.UI.TextAlignment -> m ()
setTextAlignment thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Alignment" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Anchor.
 -}
setTextAnchor :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> KRPCHS.UI.TextAnchor -> m ()
setTextAnchor thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Anchor" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Character size.
 -}
setTextCharacterSize :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> Float -> m ()
setTextCharacterSize thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_CharacterSize" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Set the color
 -}
setTextColor :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> (Double, Double, Double) -> m ()
setTextColor thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Color" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The text string
 -}
setTextContent :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> Data.Text.Text -> m ()
setTextContent thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Content" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Name of the font
 -}
setTextFont :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> Data.Text.Text -> m ()
setTextFont thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Font" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Line spacing.
 -}
setTextLineSpacing :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> Float -> m ()
setTextLineSpacing thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_LineSpacing" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Material used to render the object.
 - Creates the material from a shader with the given name.
 -}
setTextMaterial :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> Data.Text.Text -> m ()
setTextMaterial thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Material" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Position of the text.
 -}
setTextPosition :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> (Double, Double, Double) -> m ()
setTextPosition thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Position" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Reference frame for the positions of the object.
 -}
setTextReferenceFrame :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> KRPCHS.SpaceCenter.ReferenceFrame -> m ()
setTextReferenceFrame thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_ReferenceFrame" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Rotation of the text as a quaternion.
 -}
setTextRotation :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> (Double, Double, Double, Double) -> m ()
setTextRotation thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Rotation" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Font size.
 -}
setTextSize :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> Data.Int.Int32 -> m ()
setTextSize thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Size" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Font style.
 -}
setTextStyle :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> KRPCHS.UI.FontStyle -> m ()
setTextStyle thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Style" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the object is visible.
 -}
setTextVisible :: (MonadIO m, MonadThrow m, MonadRPC m) => KRPCHS.Drawing.Text -> Bool -> m ()
setTextVisible thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Visible" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

