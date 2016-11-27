module KRPCHS.Drawing
( Line
, Polygon
, Text
, addDirection
, addDirectionReq
, addDirectionStream
, addDirectionStreamReq
, addLine
, addLineReq
, addLineStream
, addLineStreamReq
, addPolygon
, addPolygonReq
, addPolygonStream
, addPolygonStreamReq
, addText
, addTextReq
, addTextStream
, addTextStreamReq
, clear
, clearReq
, lineRemove
, lineRemoveReq
, getLineColor
, getLineColorReq
, getLineColorStream
, getLineColorStreamReq
, getLineEnd
, getLineEndReq
, getLineEndStream
, getLineEndStreamReq
, getLineMaterial
, getLineMaterialReq
, getLineMaterialStream
, getLineMaterialStreamReq
, getLineReferenceFrame
, getLineReferenceFrameReq
, getLineReferenceFrameStream
, getLineReferenceFrameStreamReq
, getLineStart
, getLineStartReq
, getLineStartStream
, getLineStartStreamReq
, getLineThickness
, getLineThicknessReq
, getLineThicknessStream
, getLineThicknessStreamReq
, getLineVisible
, getLineVisibleReq
, getLineVisibleStream
, getLineVisibleStreamReq
, setLineColor
, setLineColorReq
, setLineEnd
, setLineEndReq
, setLineMaterial
, setLineMaterialReq
, setLineReferenceFrame
, setLineReferenceFrameReq
, setLineStart
, setLineStartReq
, setLineThickness
, setLineThicknessReq
, setLineVisible
, setLineVisibleReq
, polygonRemove
, polygonRemoveReq
, getPolygonColor
, getPolygonColorReq
, getPolygonColorStream
, getPolygonColorStreamReq
, getPolygonMaterial
, getPolygonMaterialReq
, getPolygonMaterialStream
, getPolygonMaterialStreamReq
, getPolygonReferenceFrame
, getPolygonReferenceFrameReq
, getPolygonReferenceFrameStream
, getPolygonReferenceFrameStreamReq
, getPolygonThickness
, getPolygonThicknessReq
, getPolygonThicknessStream
, getPolygonThicknessStreamReq
, getPolygonVertices
, getPolygonVerticesReq
, getPolygonVerticesStream
, getPolygonVerticesStreamReq
, getPolygonVisible
, getPolygonVisibleReq
, getPolygonVisibleStream
, getPolygonVisibleStreamReq
, setPolygonColor
, setPolygonColorReq
, setPolygonMaterial
, setPolygonMaterialReq
, setPolygonReferenceFrame
, setPolygonReferenceFrameReq
, setPolygonThickness
, setPolygonThicknessReq
, setPolygonVertices
, setPolygonVerticesReq
, setPolygonVisible
, setPolygonVisibleReq
, textRemove
, textRemoveReq
, getTextAlignment
, getTextAlignmentReq
, getTextAlignmentStream
, getTextAlignmentStreamReq
, getTextAnchor
, getTextAnchorReq
, getTextAnchorStream
, getTextAnchorStreamReq
, getTextAvailableFonts
, getTextAvailableFontsReq
, getTextAvailableFontsStream
, getTextAvailableFontsStreamReq
, getTextCharacterSize
, getTextCharacterSizeReq
, getTextCharacterSizeStream
, getTextCharacterSizeStreamReq
, getTextColor
, getTextColorReq
, getTextColorStream
, getTextColorStreamReq
, getTextContent
, getTextContentReq
, getTextContentStream
, getTextContentStreamReq
, getTextFont
, getTextFontReq
, getTextFontStream
, getTextFontStreamReq
, getTextLineSpacing
, getTextLineSpacingReq
, getTextLineSpacingStream
, getTextLineSpacingStreamReq
, getTextMaterial
, getTextMaterialReq
, getTextMaterialStream
, getTextMaterialStreamReq
, getTextPosition
, getTextPositionReq
, getTextPositionStream
, getTextPositionStreamReq
, getTextReferenceFrame
, getTextReferenceFrameReq
, getTextReferenceFrameStream
, getTextReferenceFrameStreamReq
, getTextRotation
, getTextRotationReq
, getTextRotationStream
, getTextRotationStreamReq
, getTextSize
, getTextSizeReq
, getTextSizeStream
, getTextSizeStreamReq
, getTextStyle
, getTextStyleReq
, getTextStyleStream
, getTextStyleStreamReq
, getTextVisible
, getTextVisibleReq
, getTextVisibleStream
, getTextVisibleStreamReq
, setTextAlignment
, setTextAlignmentReq
, setTextAnchor
, setTextAnchorReq
, setTextCharacterSize
, setTextCharacterSizeReq
, setTextColor
, setTextColorReq
, setTextContent
, setTextContentReq
, setTextFont
, setTextFontReq
, setTextLineSpacing
, setTextLineSpacingReq
, setTextMaterial
, setTextMaterialReq
, setTextPosition
, setTextPositionReq
, setTextReferenceFrame
, setTextReferenceFrameReq
, setTextRotation
, setTextRotationReq
, setTextSize
, setTextSizeReq
, setTextStyle
, setTextStyleReq
, setTextVisible
, setTextVisibleReq
) where

import qualified Data.Int
import qualified Data.Text
import qualified KRPCHS.SpaceCenter
import qualified KRPCHS.UI

import KRPCHS.Internal.Requests
import KRPCHS.Internal.Requests.Call
import KRPCHS.Internal.Requests.Stream
import KRPCHS.Internal.SerializeUtils


{-|
A line. Created using <see cref="M:Drawing.AddLine" />.
 -}
newtype Line = Line { lineId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Line where
    encodePb = encodePb . lineId

instance PbDecodable Line where
    decodePb b = Line <$> decodePb b

instance KRPCResponseExtractable Line

{-|
A polygon. Created using <see cref="M:Drawing.AddPolygon" />.
 -}
newtype Polygon = Polygon { polygonId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Polygon where
    encodePb = encodePb . polygonId

instance PbDecodable Polygon where
    decodePb b = Polygon <$> decodePb b

instance KRPCResponseExtractable Polygon

{-|
Text. Created using <see cref="M:Drawing.AddText" />.
 -}
newtype Text = Text { textId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Text where
    encodePb = encodePb . textId

instance PbDecodable Text where
    decodePb b = Text <$> decodePb b

instance KRPCResponseExtractable Text



{-|
Draw a direction vector in the scene, from the center of mass of the active vessel.<param name="direction">Direction to draw the line in.<param name="referenceFrame">Reference frame that the direction is in.<param name="length">The length of the line.<param name="visible">Whether the line is visible.
 -}
addDirectionReq :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> Float -> Bool -> KRPCCallReq (KRPCHS.Drawing.Line)
addDirectionReq directionArg referenceFrameArg lengthArg visibleArg = makeCallReq "Drawing" "AddDirection" [makeArgument 0 directionArg, makeArgument 1 referenceFrameArg, makeArgument 2 lengthArg, makeArgument 3 visibleArg]

addDirection :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> Float -> Bool -> RPCContext (KRPCHS.Drawing.Line)
addDirection directionArg referenceFrameArg lengthArg visibleArg = simpleRequest $ addDirectionReq directionArg referenceFrameArg lengthArg visibleArg

addDirectionStreamReq :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> Float -> Bool -> KRPCStreamReq (KRPCHS.Drawing.Line)
addDirectionStreamReq directionArg referenceFrameArg lengthArg visibleArg = makeStreamReq $ addDirectionReq directionArg referenceFrameArg lengthArg visibleArg

addDirectionStream :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> Float -> Bool -> RPCContext (KRPCStream (KRPCHS.Drawing.Line))
addDirectionStream directionArg referenceFrameArg lengthArg visibleArg = requestAddStream $ addDirectionStreamReq directionArg referenceFrameArg lengthArg visibleArg 

{-|
Draw a line in the scene.<param name="start">Position of the start of the line.<param name="end">Position of the end of the line.<param name="referenceFrame">Reference frame that the positions are in.<param name="visible">Whether the line is visible.
 -}
addLineReq :: (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> Bool -> KRPCCallReq (KRPCHS.Drawing.Line)
addLineReq startArg endArg referenceFrameArg visibleArg = makeCallReq "Drawing" "AddLine" [makeArgument 0 startArg, makeArgument 1 endArg, makeArgument 2 referenceFrameArg, makeArgument 3 visibleArg]

addLine :: (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> Bool -> RPCContext (KRPCHS.Drawing.Line)
addLine startArg endArg referenceFrameArg visibleArg = simpleRequest $ addLineReq startArg endArg referenceFrameArg visibleArg

addLineStreamReq :: (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> Bool -> KRPCStreamReq (KRPCHS.Drawing.Line)
addLineStreamReq startArg endArg referenceFrameArg visibleArg = makeStreamReq $ addLineReq startArg endArg referenceFrameArg visibleArg

addLineStream :: (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> Bool -> RPCContext (KRPCStream (KRPCHS.Drawing.Line))
addLineStream startArg endArg referenceFrameArg visibleArg = requestAddStream $ addLineStreamReq startArg endArg referenceFrameArg visibleArg 

{-|
Draw a polygon in the scene, defined by a list of vertices.<param name="vertices">Vertices of the polygon.<param name="referenceFrame">Reference frame that the vertices are in.<param name="visible">Whether the polygon is visible.
 -}
addPolygonReq :: [(Double, Double, Double)] -> KRPCHS.SpaceCenter.ReferenceFrame -> Bool -> KRPCCallReq (KRPCHS.Drawing.Polygon)
addPolygonReq verticesArg referenceFrameArg visibleArg = makeCallReq "Drawing" "AddPolygon" [makeArgument 0 verticesArg, makeArgument 1 referenceFrameArg, makeArgument 2 visibleArg]

addPolygon :: [(Double, Double, Double)] -> KRPCHS.SpaceCenter.ReferenceFrame -> Bool -> RPCContext (KRPCHS.Drawing.Polygon)
addPolygon verticesArg referenceFrameArg visibleArg = simpleRequest $ addPolygonReq verticesArg referenceFrameArg visibleArg

addPolygonStreamReq :: [(Double, Double, Double)] -> KRPCHS.SpaceCenter.ReferenceFrame -> Bool -> KRPCStreamReq (KRPCHS.Drawing.Polygon)
addPolygonStreamReq verticesArg referenceFrameArg visibleArg = makeStreamReq $ addPolygonReq verticesArg referenceFrameArg visibleArg

addPolygonStream :: [(Double, Double, Double)] -> KRPCHS.SpaceCenter.ReferenceFrame -> Bool -> RPCContext (KRPCStream (KRPCHS.Drawing.Polygon))
addPolygonStream verticesArg referenceFrameArg visibleArg = requestAddStream $ addPolygonStreamReq verticesArg referenceFrameArg visibleArg 

{-|
Draw text in the scene.<param name="text">The string to draw.<param name="referenceFrame">Reference frame that the text position is in.<param name="position">Position of the text.<param name="rotation">Rotation of the text, as a quaternion.<param name="visible">Whether the text is visible.
 -}
addTextReq :: Data.Text.Text -> KRPCHS.SpaceCenter.ReferenceFrame -> (Double, Double, Double) -> (Double, Double, Double, Double) -> Bool -> KRPCCallReq (KRPCHS.Drawing.Text)
addTextReq textArg referenceFrameArg positionArg rotationArg visibleArg = makeCallReq "Drawing" "AddText" [makeArgument 0 textArg, makeArgument 1 referenceFrameArg, makeArgument 2 positionArg, makeArgument 3 rotationArg, makeArgument 4 visibleArg]

addText :: Data.Text.Text -> KRPCHS.SpaceCenter.ReferenceFrame -> (Double, Double, Double) -> (Double, Double, Double, Double) -> Bool -> RPCContext (KRPCHS.Drawing.Text)
addText textArg referenceFrameArg positionArg rotationArg visibleArg = simpleRequest $ addTextReq textArg referenceFrameArg positionArg rotationArg visibleArg

addTextStreamReq :: Data.Text.Text -> KRPCHS.SpaceCenter.ReferenceFrame -> (Double, Double, Double) -> (Double, Double, Double, Double) -> Bool -> KRPCStreamReq (KRPCHS.Drawing.Text)
addTextStreamReq textArg referenceFrameArg positionArg rotationArg visibleArg = makeStreamReq $ addTextReq textArg referenceFrameArg positionArg rotationArg visibleArg

addTextStream :: Data.Text.Text -> KRPCHS.SpaceCenter.ReferenceFrame -> (Double, Double, Double) -> (Double, Double, Double, Double) -> Bool -> RPCContext (KRPCStream (KRPCHS.Drawing.Text))
addTextStream textArg referenceFrameArg positionArg rotationArg visibleArg = requestAddStream $ addTextStreamReq textArg referenceFrameArg positionArg rotationArg visibleArg 

{-|
Remove all objects being drawn.<param name="clientOnly">If true, only remove objects created by the calling client.
 -}
clearReq :: Bool -> KRPCCallReq ()
clearReq clientOnlyArg = makeCallReq "Drawing" "Clear" [makeArgument 0 clientOnlyArg]

clear :: Bool -> RPCContext ()
clear clientOnlyArg = simpleRequest $ clearReq clientOnlyArg 

{-|
Remove the object.
 -}
lineRemoveReq :: KRPCHS.Drawing.Line -> KRPCCallReq ()
lineRemoveReq thisArg = makeCallReq "Drawing" "Line_Remove" [makeArgument 0 thisArg]

lineRemove :: KRPCHS.Drawing.Line -> RPCContext ()
lineRemove thisArg = simpleRequest $ lineRemoveReq thisArg 

{-|
Set the color
 -}
getLineColorReq :: KRPCHS.Drawing.Line -> KRPCCallReq ((Double, Double, Double))
getLineColorReq thisArg = makeCallReq "Drawing" "Line_get_Color" [makeArgument 0 thisArg]

getLineColor :: KRPCHS.Drawing.Line -> RPCContext ((Double, Double, Double))
getLineColor thisArg = simpleRequest $ getLineColorReq thisArg

getLineColorStreamReq :: KRPCHS.Drawing.Line -> KRPCStreamReq ((Double, Double, Double))
getLineColorStreamReq thisArg = makeStreamReq $ getLineColorReq thisArg

getLineColorStream :: KRPCHS.Drawing.Line -> RPCContext (KRPCStream ((Double, Double, Double)))
getLineColorStream thisArg = requestAddStream $ getLineColorStreamReq thisArg 

{-|
End position of the line.
 -}
getLineEndReq :: KRPCHS.Drawing.Line -> KRPCCallReq ((Double, Double, Double))
getLineEndReq thisArg = makeCallReq "Drawing" "Line_get_End" [makeArgument 0 thisArg]

getLineEnd :: KRPCHS.Drawing.Line -> RPCContext ((Double, Double, Double))
getLineEnd thisArg = simpleRequest $ getLineEndReq thisArg

getLineEndStreamReq :: KRPCHS.Drawing.Line -> KRPCStreamReq ((Double, Double, Double))
getLineEndStreamReq thisArg = makeStreamReq $ getLineEndReq thisArg

getLineEndStream :: KRPCHS.Drawing.Line -> RPCContext (KRPCStream ((Double, Double, Double)))
getLineEndStream thisArg = requestAddStream $ getLineEndStreamReq thisArg 

{-|
Material used to render the object.
Creates the material from a shader with the given name.
 -}
getLineMaterialReq :: KRPCHS.Drawing.Line -> KRPCCallReq (Data.Text.Text)
getLineMaterialReq thisArg = makeCallReq "Drawing" "Line_get_Material" [makeArgument 0 thisArg]

getLineMaterial :: KRPCHS.Drawing.Line -> RPCContext (Data.Text.Text)
getLineMaterial thisArg = simpleRequest $ getLineMaterialReq thisArg

getLineMaterialStreamReq :: KRPCHS.Drawing.Line -> KRPCStreamReq (Data.Text.Text)
getLineMaterialStreamReq thisArg = makeStreamReq $ getLineMaterialReq thisArg

getLineMaterialStream :: KRPCHS.Drawing.Line -> RPCContext (KRPCStream (Data.Text.Text))
getLineMaterialStream thisArg = requestAddStream $ getLineMaterialStreamReq thisArg 

{-|
Reference frame for the positions of the object.
 -}
getLineReferenceFrameReq :: KRPCHS.Drawing.Line -> KRPCCallReq (KRPCHS.SpaceCenter.ReferenceFrame)
getLineReferenceFrameReq thisArg = makeCallReq "Drawing" "Line_get_ReferenceFrame" [makeArgument 0 thisArg]

getLineReferenceFrame :: KRPCHS.Drawing.Line -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getLineReferenceFrame thisArg = simpleRequest $ getLineReferenceFrameReq thisArg

getLineReferenceFrameStreamReq :: KRPCHS.Drawing.Line -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getLineReferenceFrameStreamReq thisArg = makeStreamReq $ getLineReferenceFrameReq thisArg

getLineReferenceFrameStream :: KRPCHS.Drawing.Line -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getLineReferenceFrameStream thisArg = requestAddStream $ getLineReferenceFrameStreamReq thisArg 

{-|
Start position of the line.
 -}
getLineStartReq :: KRPCHS.Drawing.Line -> KRPCCallReq ((Double, Double, Double))
getLineStartReq thisArg = makeCallReq "Drawing" "Line_get_Start" [makeArgument 0 thisArg]

getLineStart :: KRPCHS.Drawing.Line -> RPCContext ((Double, Double, Double))
getLineStart thisArg = simpleRequest $ getLineStartReq thisArg

getLineStartStreamReq :: KRPCHS.Drawing.Line -> KRPCStreamReq ((Double, Double, Double))
getLineStartStreamReq thisArg = makeStreamReq $ getLineStartReq thisArg

getLineStartStream :: KRPCHS.Drawing.Line -> RPCContext (KRPCStream ((Double, Double, Double)))
getLineStartStream thisArg = requestAddStream $ getLineStartStreamReq thisArg 

{-|
Set the thickness
 -}
getLineThicknessReq :: KRPCHS.Drawing.Line -> KRPCCallReq (Float)
getLineThicknessReq thisArg = makeCallReq "Drawing" "Line_get_Thickness" [makeArgument 0 thisArg]

getLineThickness :: KRPCHS.Drawing.Line -> RPCContext (Float)
getLineThickness thisArg = simpleRequest $ getLineThicknessReq thisArg

getLineThicknessStreamReq :: KRPCHS.Drawing.Line -> KRPCStreamReq (Float)
getLineThicknessStreamReq thisArg = makeStreamReq $ getLineThicknessReq thisArg

getLineThicknessStream :: KRPCHS.Drawing.Line -> RPCContext (KRPCStream (Float))
getLineThicknessStream thisArg = requestAddStream $ getLineThicknessStreamReq thisArg 

{-|
Whether the object is visible.
 -}
getLineVisibleReq :: KRPCHS.Drawing.Line -> KRPCCallReq (Bool)
getLineVisibleReq thisArg = makeCallReq "Drawing" "Line_get_Visible" [makeArgument 0 thisArg]

getLineVisible :: KRPCHS.Drawing.Line -> RPCContext (Bool)
getLineVisible thisArg = simpleRequest $ getLineVisibleReq thisArg

getLineVisibleStreamReq :: KRPCHS.Drawing.Line -> KRPCStreamReq (Bool)
getLineVisibleStreamReq thisArg = makeStreamReq $ getLineVisibleReq thisArg

getLineVisibleStream :: KRPCHS.Drawing.Line -> RPCContext (KRPCStream (Bool))
getLineVisibleStream thisArg = requestAddStream $ getLineVisibleStreamReq thisArg 

{-|
Set the color
 -}
setLineColorReq :: KRPCHS.Drawing.Line -> (Double, Double, Double) -> KRPCCallReq ()
setLineColorReq thisArg valueArg = makeCallReq "Drawing" "Line_set_Color" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setLineColor :: KRPCHS.Drawing.Line -> (Double, Double, Double) -> RPCContext ()
setLineColor thisArg valueArg = simpleRequest $ setLineColorReq thisArg valueArg 

{-|
End position of the line.
 -}
setLineEndReq :: KRPCHS.Drawing.Line -> (Double, Double, Double) -> KRPCCallReq ()
setLineEndReq thisArg valueArg = makeCallReq "Drawing" "Line_set_End" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setLineEnd :: KRPCHS.Drawing.Line -> (Double, Double, Double) -> RPCContext ()
setLineEnd thisArg valueArg = simpleRequest $ setLineEndReq thisArg valueArg 

{-|
Material used to render the object.
Creates the material from a shader with the given name.
 -}
setLineMaterialReq :: KRPCHS.Drawing.Line -> Data.Text.Text -> KRPCCallReq ()
setLineMaterialReq thisArg valueArg = makeCallReq "Drawing" "Line_set_Material" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setLineMaterial :: KRPCHS.Drawing.Line -> Data.Text.Text -> RPCContext ()
setLineMaterial thisArg valueArg = simpleRequest $ setLineMaterialReq thisArg valueArg 

{-|
Reference frame for the positions of the object.
 -}
setLineReferenceFrameReq :: KRPCHS.Drawing.Line -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ()
setLineReferenceFrameReq thisArg valueArg = makeCallReq "Drawing" "Line_set_ReferenceFrame" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setLineReferenceFrame :: KRPCHS.Drawing.Line -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ()
setLineReferenceFrame thisArg valueArg = simpleRequest $ setLineReferenceFrameReq thisArg valueArg 

{-|
Start position of the line.
 -}
setLineStartReq :: KRPCHS.Drawing.Line -> (Double, Double, Double) -> KRPCCallReq ()
setLineStartReq thisArg valueArg = makeCallReq "Drawing" "Line_set_Start" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setLineStart :: KRPCHS.Drawing.Line -> (Double, Double, Double) -> RPCContext ()
setLineStart thisArg valueArg = simpleRequest $ setLineStartReq thisArg valueArg 

{-|
Set the thickness
 -}
setLineThicknessReq :: KRPCHS.Drawing.Line -> Float -> KRPCCallReq ()
setLineThicknessReq thisArg valueArg = makeCallReq "Drawing" "Line_set_Thickness" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setLineThickness :: KRPCHS.Drawing.Line -> Float -> RPCContext ()
setLineThickness thisArg valueArg = simpleRequest $ setLineThicknessReq thisArg valueArg 

{-|
Whether the object is visible.
 -}
setLineVisibleReq :: KRPCHS.Drawing.Line -> Bool -> KRPCCallReq ()
setLineVisibleReq thisArg valueArg = makeCallReq "Drawing" "Line_set_Visible" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setLineVisible :: KRPCHS.Drawing.Line -> Bool -> RPCContext ()
setLineVisible thisArg valueArg = simpleRequest $ setLineVisibleReq thisArg valueArg 

{-|
Remove the object.
 -}
polygonRemoveReq :: KRPCHS.Drawing.Polygon -> KRPCCallReq ()
polygonRemoveReq thisArg = makeCallReq "Drawing" "Polygon_Remove" [makeArgument 0 thisArg]

polygonRemove :: KRPCHS.Drawing.Polygon -> RPCContext ()
polygonRemove thisArg = simpleRequest $ polygonRemoveReq thisArg 

{-|
Set the color
 -}
getPolygonColorReq :: KRPCHS.Drawing.Polygon -> KRPCCallReq ((Double, Double, Double))
getPolygonColorReq thisArg = makeCallReq "Drawing" "Polygon_get_Color" [makeArgument 0 thisArg]

getPolygonColor :: KRPCHS.Drawing.Polygon -> RPCContext ((Double, Double, Double))
getPolygonColor thisArg = simpleRequest $ getPolygonColorReq thisArg

getPolygonColorStreamReq :: KRPCHS.Drawing.Polygon -> KRPCStreamReq ((Double, Double, Double))
getPolygonColorStreamReq thisArg = makeStreamReq $ getPolygonColorReq thisArg

getPolygonColorStream :: KRPCHS.Drawing.Polygon -> RPCContext (KRPCStream ((Double, Double, Double)))
getPolygonColorStream thisArg = requestAddStream $ getPolygonColorStreamReq thisArg 

{-|
Material used to render the object.
Creates the material from a shader with the given name.
 -}
getPolygonMaterialReq :: KRPCHS.Drawing.Polygon -> KRPCCallReq (Data.Text.Text)
getPolygonMaterialReq thisArg = makeCallReq "Drawing" "Polygon_get_Material" [makeArgument 0 thisArg]

getPolygonMaterial :: KRPCHS.Drawing.Polygon -> RPCContext (Data.Text.Text)
getPolygonMaterial thisArg = simpleRequest $ getPolygonMaterialReq thisArg

getPolygonMaterialStreamReq :: KRPCHS.Drawing.Polygon -> KRPCStreamReq (Data.Text.Text)
getPolygonMaterialStreamReq thisArg = makeStreamReq $ getPolygonMaterialReq thisArg

getPolygonMaterialStream :: KRPCHS.Drawing.Polygon -> RPCContext (KRPCStream (Data.Text.Text))
getPolygonMaterialStream thisArg = requestAddStream $ getPolygonMaterialStreamReq thisArg 

{-|
Reference frame for the positions of the object.
 -}
getPolygonReferenceFrameReq :: KRPCHS.Drawing.Polygon -> KRPCCallReq (KRPCHS.SpaceCenter.ReferenceFrame)
getPolygonReferenceFrameReq thisArg = makeCallReq "Drawing" "Polygon_get_ReferenceFrame" [makeArgument 0 thisArg]

getPolygonReferenceFrame :: KRPCHS.Drawing.Polygon -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getPolygonReferenceFrame thisArg = simpleRequest $ getPolygonReferenceFrameReq thisArg

getPolygonReferenceFrameStreamReq :: KRPCHS.Drawing.Polygon -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getPolygonReferenceFrameStreamReq thisArg = makeStreamReq $ getPolygonReferenceFrameReq thisArg

getPolygonReferenceFrameStream :: KRPCHS.Drawing.Polygon -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getPolygonReferenceFrameStream thisArg = requestAddStream $ getPolygonReferenceFrameStreamReq thisArg 

{-|
Set the thickness
 -}
getPolygonThicknessReq :: KRPCHS.Drawing.Polygon -> KRPCCallReq (Float)
getPolygonThicknessReq thisArg = makeCallReq "Drawing" "Polygon_get_Thickness" [makeArgument 0 thisArg]

getPolygonThickness :: KRPCHS.Drawing.Polygon -> RPCContext (Float)
getPolygonThickness thisArg = simpleRequest $ getPolygonThicknessReq thisArg

getPolygonThicknessStreamReq :: KRPCHS.Drawing.Polygon -> KRPCStreamReq (Float)
getPolygonThicknessStreamReq thisArg = makeStreamReq $ getPolygonThicknessReq thisArg

getPolygonThicknessStream :: KRPCHS.Drawing.Polygon -> RPCContext (KRPCStream (Float))
getPolygonThicknessStream thisArg = requestAddStream $ getPolygonThicknessStreamReq thisArg 

{-|
Vertices for the polygon.
 -}
getPolygonVerticesReq :: KRPCHS.Drawing.Polygon -> KRPCCallReq ([(Double, Double, Double)])
getPolygonVerticesReq thisArg = makeCallReq "Drawing" "Polygon_get_Vertices" [makeArgument 0 thisArg]

getPolygonVertices :: KRPCHS.Drawing.Polygon -> RPCContext ([(Double, Double, Double)])
getPolygonVertices thisArg = simpleRequest $ getPolygonVerticesReq thisArg

getPolygonVerticesStreamReq :: KRPCHS.Drawing.Polygon -> KRPCStreamReq ([(Double, Double, Double)])
getPolygonVerticesStreamReq thisArg = makeStreamReq $ getPolygonVerticesReq thisArg

getPolygonVerticesStream :: KRPCHS.Drawing.Polygon -> RPCContext (KRPCStream ([(Double, Double, Double)]))
getPolygonVerticesStream thisArg = requestAddStream $ getPolygonVerticesStreamReq thisArg 

{-|
Whether the object is visible.
 -}
getPolygonVisibleReq :: KRPCHS.Drawing.Polygon -> KRPCCallReq (Bool)
getPolygonVisibleReq thisArg = makeCallReq "Drawing" "Polygon_get_Visible" [makeArgument 0 thisArg]

getPolygonVisible :: KRPCHS.Drawing.Polygon -> RPCContext (Bool)
getPolygonVisible thisArg = simpleRequest $ getPolygonVisibleReq thisArg

getPolygonVisibleStreamReq :: KRPCHS.Drawing.Polygon -> KRPCStreamReq (Bool)
getPolygonVisibleStreamReq thisArg = makeStreamReq $ getPolygonVisibleReq thisArg

getPolygonVisibleStream :: KRPCHS.Drawing.Polygon -> RPCContext (KRPCStream (Bool))
getPolygonVisibleStream thisArg = requestAddStream $ getPolygonVisibleStreamReq thisArg 

{-|
Set the color
 -}
setPolygonColorReq :: KRPCHS.Drawing.Polygon -> (Double, Double, Double) -> KRPCCallReq ()
setPolygonColorReq thisArg valueArg = makeCallReq "Drawing" "Polygon_set_Color" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setPolygonColor :: KRPCHS.Drawing.Polygon -> (Double, Double, Double) -> RPCContext ()
setPolygonColor thisArg valueArg = simpleRequest $ setPolygonColorReq thisArg valueArg 

{-|
Material used to render the object.
Creates the material from a shader with the given name.
 -}
setPolygonMaterialReq :: KRPCHS.Drawing.Polygon -> Data.Text.Text -> KRPCCallReq ()
setPolygonMaterialReq thisArg valueArg = makeCallReq "Drawing" "Polygon_set_Material" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setPolygonMaterial :: KRPCHS.Drawing.Polygon -> Data.Text.Text -> RPCContext ()
setPolygonMaterial thisArg valueArg = simpleRequest $ setPolygonMaterialReq thisArg valueArg 

{-|
Reference frame for the positions of the object.
 -}
setPolygonReferenceFrameReq :: KRPCHS.Drawing.Polygon -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ()
setPolygonReferenceFrameReq thisArg valueArg = makeCallReq "Drawing" "Polygon_set_ReferenceFrame" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setPolygonReferenceFrame :: KRPCHS.Drawing.Polygon -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ()
setPolygonReferenceFrame thisArg valueArg = simpleRequest $ setPolygonReferenceFrameReq thisArg valueArg 

{-|
Set the thickness
 -}
setPolygonThicknessReq :: KRPCHS.Drawing.Polygon -> Float -> KRPCCallReq ()
setPolygonThicknessReq thisArg valueArg = makeCallReq "Drawing" "Polygon_set_Thickness" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setPolygonThickness :: KRPCHS.Drawing.Polygon -> Float -> RPCContext ()
setPolygonThickness thisArg valueArg = simpleRequest $ setPolygonThicknessReq thisArg valueArg 

{-|
Vertices for the polygon.
 -}
setPolygonVerticesReq :: KRPCHS.Drawing.Polygon -> [(Double, Double, Double)] -> KRPCCallReq ()
setPolygonVerticesReq thisArg valueArg = makeCallReq "Drawing" "Polygon_set_Vertices" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setPolygonVertices :: KRPCHS.Drawing.Polygon -> [(Double, Double, Double)] -> RPCContext ()
setPolygonVertices thisArg valueArg = simpleRequest $ setPolygonVerticesReq thisArg valueArg 

{-|
Whether the object is visible.
 -}
setPolygonVisibleReq :: KRPCHS.Drawing.Polygon -> Bool -> KRPCCallReq ()
setPolygonVisibleReq thisArg valueArg = makeCallReq "Drawing" "Polygon_set_Visible" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setPolygonVisible :: KRPCHS.Drawing.Polygon -> Bool -> RPCContext ()
setPolygonVisible thisArg valueArg = simpleRequest $ setPolygonVisibleReq thisArg valueArg 

{-|
Remove the object.
 -}
textRemoveReq :: KRPCHS.Drawing.Text -> KRPCCallReq ()
textRemoveReq thisArg = makeCallReq "Drawing" "Text_Remove" [makeArgument 0 thisArg]

textRemove :: KRPCHS.Drawing.Text -> RPCContext ()
textRemove thisArg = simpleRequest $ textRemoveReq thisArg 

{-|
Alignment.
 -}
getTextAlignmentReq :: KRPCHS.Drawing.Text -> KRPCCallReq (KRPCHS.UI.TextAlignment)
getTextAlignmentReq thisArg = makeCallReq "Drawing" "Text_get_Alignment" [makeArgument 0 thisArg]

getTextAlignment :: KRPCHS.Drawing.Text -> RPCContext (KRPCHS.UI.TextAlignment)
getTextAlignment thisArg = simpleRequest $ getTextAlignmentReq thisArg

getTextAlignmentStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq (KRPCHS.UI.TextAlignment)
getTextAlignmentStreamReq thisArg = makeStreamReq $ getTextAlignmentReq thisArg

getTextAlignmentStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (KRPCHS.UI.TextAlignment))
getTextAlignmentStream thisArg = requestAddStream $ getTextAlignmentStreamReq thisArg 

{-|
Anchor.
 -}
getTextAnchorReq :: KRPCHS.Drawing.Text -> KRPCCallReq (KRPCHS.UI.TextAnchor)
getTextAnchorReq thisArg = makeCallReq "Drawing" "Text_get_Anchor" [makeArgument 0 thisArg]

getTextAnchor :: KRPCHS.Drawing.Text -> RPCContext (KRPCHS.UI.TextAnchor)
getTextAnchor thisArg = simpleRequest $ getTextAnchorReq thisArg

getTextAnchorStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq (KRPCHS.UI.TextAnchor)
getTextAnchorStreamReq thisArg = makeStreamReq $ getTextAnchorReq thisArg

getTextAnchorStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (KRPCHS.UI.TextAnchor))
getTextAnchorStream thisArg = requestAddStream $ getTextAnchorStreamReq thisArg 

{-|
A list of all available fonts.
 -}
getTextAvailableFontsReq :: KRPCHS.Drawing.Text -> KRPCCallReq ([Data.Text.Text])
getTextAvailableFontsReq thisArg = makeCallReq "Drawing" "Text_get_AvailableFonts" [makeArgument 0 thisArg]

getTextAvailableFonts :: KRPCHS.Drawing.Text -> RPCContext ([Data.Text.Text])
getTextAvailableFonts thisArg = simpleRequest $ getTextAvailableFontsReq thisArg

getTextAvailableFontsStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq ([Data.Text.Text])
getTextAvailableFontsStreamReq thisArg = makeStreamReq $ getTextAvailableFontsReq thisArg

getTextAvailableFontsStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream ([Data.Text.Text]))
getTextAvailableFontsStream thisArg = requestAddStream $ getTextAvailableFontsStreamReq thisArg 

{-|
Character size.
 -}
getTextCharacterSizeReq :: KRPCHS.Drawing.Text -> KRPCCallReq (Float)
getTextCharacterSizeReq thisArg = makeCallReq "Drawing" "Text_get_CharacterSize" [makeArgument 0 thisArg]

getTextCharacterSize :: KRPCHS.Drawing.Text -> RPCContext (Float)
getTextCharacterSize thisArg = simpleRequest $ getTextCharacterSizeReq thisArg

getTextCharacterSizeStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq (Float)
getTextCharacterSizeStreamReq thisArg = makeStreamReq $ getTextCharacterSizeReq thisArg

getTextCharacterSizeStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (Float))
getTextCharacterSizeStream thisArg = requestAddStream $ getTextCharacterSizeStreamReq thisArg 

{-|
Set the color
 -}
getTextColorReq :: KRPCHS.Drawing.Text -> KRPCCallReq ((Double, Double, Double))
getTextColorReq thisArg = makeCallReq "Drawing" "Text_get_Color" [makeArgument 0 thisArg]

getTextColor :: KRPCHS.Drawing.Text -> RPCContext ((Double, Double, Double))
getTextColor thisArg = simpleRequest $ getTextColorReq thisArg

getTextColorStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq ((Double, Double, Double))
getTextColorStreamReq thisArg = makeStreamReq $ getTextColorReq thisArg

getTextColorStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream ((Double, Double, Double)))
getTextColorStream thisArg = requestAddStream $ getTextColorStreamReq thisArg 

{-|
The text string
 -}
getTextContentReq :: KRPCHS.Drawing.Text -> KRPCCallReq (Data.Text.Text)
getTextContentReq thisArg = makeCallReq "Drawing" "Text_get_Content" [makeArgument 0 thisArg]

getTextContent :: KRPCHS.Drawing.Text -> RPCContext (Data.Text.Text)
getTextContent thisArg = simpleRequest $ getTextContentReq thisArg

getTextContentStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq (Data.Text.Text)
getTextContentStreamReq thisArg = makeStreamReq $ getTextContentReq thisArg

getTextContentStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (Data.Text.Text))
getTextContentStream thisArg = requestAddStream $ getTextContentStreamReq thisArg 

{-|
Name of the font
 -}
getTextFontReq :: KRPCHS.Drawing.Text -> KRPCCallReq (Data.Text.Text)
getTextFontReq thisArg = makeCallReq "Drawing" "Text_get_Font" [makeArgument 0 thisArg]

getTextFont :: KRPCHS.Drawing.Text -> RPCContext (Data.Text.Text)
getTextFont thisArg = simpleRequest $ getTextFontReq thisArg

getTextFontStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq (Data.Text.Text)
getTextFontStreamReq thisArg = makeStreamReq $ getTextFontReq thisArg

getTextFontStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (Data.Text.Text))
getTextFontStream thisArg = requestAddStream $ getTextFontStreamReq thisArg 

{-|
Line spacing.
 -}
getTextLineSpacingReq :: KRPCHS.Drawing.Text -> KRPCCallReq (Float)
getTextLineSpacingReq thisArg = makeCallReq "Drawing" "Text_get_LineSpacing" [makeArgument 0 thisArg]

getTextLineSpacing :: KRPCHS.Drawing.Text -> RPCContext (Float)
getTextLineSpacing thisArg = simpleRequest $ getTextLineSpacingReq thisArg

getTextLineSpacingStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq (Float)
getTextLineSpacingStreamReq thisArg = makeStreamReq $ getTextLineSpacingReq thisArg

getTextLineSpacingStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (Float))
getTextLineSpacingStream thisArg = requestAddStream $ getTextLineSpacingStreamReq thisArg 

{-|
Material used to render the object.
Creates the material from a shader with the given name.
 -}
getTextMaterialReq :: KRPCHS.Drawing.Text -> KRPCCallReq (Data.Text.Text)
getTextMaterialReq thisArg = makeCallReq "Drawing" "Text_get_Material" [makeArgument 0 thisArg]

getTextMaterial :: KRPCHS.Drawing.Text -> RPCContext (Data.Text.Text)
getTextMaterial thisArg = simpleRequest $ getTextMaterialReq thisArg

getTextMaterialStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq (Data.Text.Text)
getTextMaterialStreamReq thisArg = makeStreamReq $ getTextMaterialReq thisArg

getTextMaterialStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (Data.Text.Text))
getTextMaterialStream thisArg = requestAddStream $ getTextMaterialStreamReq thisArg 

{-|
Position of the text.
 -}
getTextPositionReq :: KRPCHS.Drawing.Text -> KRPCCallReq ((Double, Double, Double))
getTextPositionReq thisArg = makeCallReq "Drawing" "Text_get_Position" [makeArgument 0 thisArg]

getTextPosition :: KRPCHS.Drawing.Text -> RPCContext ((Double, Double, Double))
getTextPosition thisArg = simpleRequest $ getTextPositionReq thisArg

getTextPositionStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq ((Double, Double, Double))
getTextPositionStreamReq thisArg = makeStreamReq $ getTextPositionReq thisArg

getTextPositionStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream ((Double, Double, Double)))
getTextPositionStream thisArg = requestAddStream $ getTextPositionStreamReq thisArg 

{-|
Reference frame for the positions of the object.
 -}
getTextReferenceFrameReq :: KRPCHS.Drawing.Text -> KRPCCallReq (KRPCHS.SpaceCenter.ReferenceFrame)
getTextReferenceFrameReq thisArg = makeCallReq "Drawing" "Text_get_ReferenceFrame" [makeArgument 0 thisArg]

getTextReferenceFrame :: KRPCHS.Drawing.Text -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getTextReferenceFrame thisArg = simpleRequest $ getTextReferenceFrameReq thisArg

getTextReferenceFrameStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame)
getTextReferenceFrameStreamReq thisArg = makeStreamReq $ getTextReferenceFrameReq thisArg

getTextReferenceFrameStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getTextReferenceFrameStream thisArg = requestAddStream $ getTextReferenceFrameStreamReq thisArg 

{-|
Rotation of the text as a quaternion.
 -}
getTextRotationReq :: KRPCHS.Drawing.Text -> KRPCCallReq ((Double, Double, Double, Double))
getTextRotationReq thisArg = makeCallReq "Drawing" "Text_get_Rotation" [makeArgument 0 thisArg]

getTextRotation :: KRPCHS.Drawing.Text -> RPCContext ((Double, Double, Double, Double))
getTextRotation thisArg = simpleRequest $ getTextRotationReq thisArg

getTextRotationStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq ((Double, Double, Double, Double))
getTextRotationStreamReq thisArg = makeStreamReq $ getTextRotationReq thisArg

getTextRotationStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
getTextRotationStream thisArg = requestAddStream $ getTextRotationStreamReq thisArg 

{-|
Font size.
 -}
getTextSizeReq :: KRPCHS.Drawing.Text -> KRPCCallReq (Data.Int.Int32)
getTextSizeReq thisArg = makeCallReq "Drawing" "Text_get_Size" [makeArgument 0 thisArg]

getTextSize :: KRPCHS.Drawing.Text -> RPCContext (Data.Int.Int32)
getTextSize thisArg = simpleRequest $ getTextSizeReq thisArg

getTextSizeStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq (Data.Int.Int32)
getTextSizeStreamReq thisArg = makeStreamReq $ getTextSizeReq thisArg

getTextSizeStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (Data.Int.Int32))
getTextSizeStream thisArg = requestAddStream $ getTextSizeStreamReq thisArg 

{-|
Font style.
 -}
getTextStyleReq :: KRPCHS.Drawing.Text -> KRPCCallReq (KRPCHS.UI.FontStyle)
getTextStyleReq thisArg = makeCallReq "Drawing" "Text_get_Style" [makeArgument 0 thisArg]

getTextStyle :: KRPCHS.Drawing.Text -> RPCContext (KRPCHS.UI.FontStyle)
getTextStyle thisArg = simpleRequest $ getTextStyleReq thisArg

getTextStyleStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq (KRPCHS.UI.FontStyle)
getTextStyleStreamReq thisArg = makeStreamReq $ getTextStyleReq thisArg

getTextStyleStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (KRPCHS.UI.FontStyle))
getTextStyleStream thisArg = requestAddStream $ getTextStyleStreamReq thisArg 

{-|
Whether the object is visible.
 -}
getTextVisibleReq :: KRPCHS.Drawing.Text -> KRPCCallReq (Bool)
getTextVisibleReq thisArg = makeCallReq "Drawing" "Text_get_Visible" [makeArgument 0 thisArg]

getTextVisible :: KRPCHS.Drawing.Text -> RPCContext (Bool)
getTextVisible thisArg = simpleRequest $ getTextVisibleReq thisArg

getTextVisibleStreamReq :: KRPCHS.Drawing.Text -> KRPCStreamReq (Bool)
getTextVisibleStreamReq thisArg = makeStreamReq $ getTextVisibleReq thisArg

getTextVisibleStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (Bool))
getTextVisibleStream thisArg = requestAddStream $ getTextVisibleStreamReq thisArg 

{-|
Alignment.
 -}
setTextAlignmentReq :: KRPCHS.Drawing.Text -> KRPCHS.UI.TextAlignment -> KRPCCallReq ()
setTextAlignmentReq thisArg valueArg = makeCallReq "Drawing" "Text_set_Alignment" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setTextAlignment :: KRPCHS.Drawing.Text -> KRPCHS.UI.TextAlignment -> RPCContext ()
setTextAlignment thisArg valueArg = simpleRequest $ setTextAlignmentReq thisArg valueArg 

{-|
Anchor.
 -}
setTextAnchorReq :: KRPCHS.Drawing.Text -> KRPCHS.UI.TextAnchor -> KRPCCallReq ()
setTextAnchorReq thisArg valueArg = makeCallReq "Drawing" "Text_set_Anchor" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setTextAnchor :: KRPCHS.Drawing.Text -> KRPCHS.UI.TextAnchor -> RPCContext ()
setTextAnchor thisArg valueArg = simpleRequest $ setTextAnchorReq thisArg valueArg 

{-|
Character size.
 -}
setTextCharacterSizeReq :: KRPCHS.Drawing.Text -> Float -> KRPCCallReq ()
setTextCharacterSizeReq thisArg valueArg = makeCallReq "Drawing" "Text_set_CharacterSize" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setTextCharacterSize :: KRPCHS.Drawing.Text -> Float -> RPCContext ()
setTextCharacterSize thisArg valueArg = simpleRequest $ setTextCharacterSizeReq thisArg valueArg 

{-|
Set the color
 -}
setTextColorReq :: KRPCHS.Drawing.Text -> (Double, Double, Double) -> KRPCCallReq ()
setTextColorReq thisArg valueArg = makeCallReq "Drawing" "Text_set_Color" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setTextColor :: KRPCHS.Drawing.Text -> (Double, Double, Double) -> RPCContext ()
setTextColor thisArg valueArg = simpleRequest $ setTextColorReq thisArg valueArg 

{-|
The text string
 -}
setTextContentReq :: KRPCHS.Drawing.Text -> Data.Text.Text -> KRPCCallReq ()
setTextContentReq thisArg valueArg = makeCallReq "Drawing" "Text_set_Content" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setTextContent :: KRPCHS.Drawing.Text -> Data.Text.Text -> RPCContext ()
setTextContent thisArg valueArg = simpleRequest $ setTextContentReq thisArg valueArg 

{-|
Name of the font
 -}
setTextFontReq :: KRPCHS.Drawing.Text -> Data.Text.Text -> KRPCCallReq ()
setTextFontReq thisArg valueArg = makeCallReq "Drawing" "Text_set_Font" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setTextFont :: KRPCHS.Drawing.Text -> Data.Text.Text -> RPCContext ()
setTextFont thisArg valueArg = simpleRequest $ setTextFontReq thisArg valueArg 

{-|
Line spacing.
 -}
setTextLineSpacingReq :: KRPCHS.Drawing.Text -> Float -> KRPCCallReq ()
setTextLineSpacingReq thisArg valueArg = makeCallReq "Drawing" "Text_set_LineSpacing" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setTextLineSpacing :: KRPCHS.Drawing.Text -> Float -> RPCContext ()
setTextLineSpacing thisArg valueArg = simpleRequest $ setTextLineSpacingReq thisArg valueArg 

{-|
Material used to render the object.
Creates the material from a shader with the given name.
 -}
setTextMaterialReq :: KRPCHS.Drawing.Text -> Data.Text.Text -> KRPCCallReq ()
setTextMaterialReq thisArg valueArg = makeCallReq "Drawing" "Text_set_Material" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setTextMaterial :: KRPCHS.Drawing.Text -> Data.Text.Text -> RPCContext ()
setTextMaterial thisArg valueArg = simpleRequest $ setTextMaterialReq thisArg valueArg 

{-|
Position of the text.
 -}
setTextPositionReq :: KRPCHS.Drawing.Text -> (Double, Double, Double) -> KRPCCallReq ()
setTextPositionReq thisArg valueArg = makeCallReq "Drawing" "Text_set_Position" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setTextPosition :: KRPCHS.Drawing.Text -> (Double, Double, Double) -> RPCContext ()
setTextPosition thisArg valueArg = simpleRequest $ setTextPositionReq thisArg valueArg 

{-|
Reference frame for the positions of the object.
 -}
setTextReferenceFrameReq :: KRPCHS.Drawing.Text -> KRPCHS.SpaceCenter.ReferenceFrame -> KRPCCallReq ()
setTextReferenceFrameReq thisArg valueArg = makeCallReq "Drawing" "Text_set_ReferenceFrame" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setTextReferenceFrame :: KRPCHS.Drawing.Text -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ()
setTextReferenceFrame thisArg valueArg = simpleRequest $ setTextReferenceFrameReq thisArg valueArg 

{-|
Rotation of the text as a quaternion.
 -}
setTextRotationReq :: KRPCHS.Drawing.Text -> (Double, Double, Double, Double) -> KRPCCallReq ()
setTextRotationReq thisArg valueArg = makeCallReq "Drawing" "Text_set_Rotation" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setTextRotation :: KRPCHS.Drawing.Text -> (Double, Double, Double, Double) -> RPCContext ()
setTextRotation thisArg valueArg = simpleRequest $ setTextRotationReq thisArg valueArg 

{-|
Font size.
 -}
setTextSizeReq :: KRPCHS.Drawing.Text -> Data.Int.Int32 -> KRPCCallReq ()
setTextSizeReq thisArg valueArg = makeCallReq "Drawing" "Text_set_Size" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setTextSize :: KRPCHS.Drawing.Text -> Data.Int.Int32 -> RPCContext ()
setTextSize thisArg valueArg = simpleRequest $ setTextSizeReq thisArg valueArg 

{-|
Font style.
 -}
setTextStyleReq :: KRPCHS.Drawing.Text -> KRPCHS.UI.FontStyle -> KRPCCallReq ()
setTextStyleReq thisArg valueArg = makeCallReq "Drawing" "Text_set_Style" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setTextStyle :: KRPCHS.Drawing.Text -> KRPCHS.UI.FontStyle -> RPCContext ()
setTextStyle thisArg valueArg = simpleRequest $ setTextStyleReq thisArg valueArg 

{-|
Whether the object is visible.
 -}
setTextVisibleReq :: KRPCHS.Drawing.Text -> Bool -> KRPCCallReq ()
setTextVisibleReq thisArg valueArg = makeCallReq "Drawing" "Text_set_Visible" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setTextVisible :: KRPCHS.Drawing.Text -> Bool -> RPCContext ()
setTextVisible thisArg valueArg = simpleRequest $ setTextVisibleReq thisArg valueArg 

