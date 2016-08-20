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
addDirection :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> Float -> Bool -> RPCContext (KRPCHS.Drawing.Line)
addDirection directionArg referenceFrameArg lengthArg visibleArg = do
    let r = makeRequest "Drawing" "AddDirection" [makeArgument 0 directionArg, makeArgument 1 referenceFrameArg, makeArgument 2 lengthArg, makeArgument 3 visibleArg]
    res <- sendRequest r
    processResponse res

addDirectionStreamReq :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> Float -> Bool -> RPCContext (KRPCStreamReq (KRPCHS.Drawing.Line))
addDirectionStreamReq directionArg referenceFrameArg lengthArg visibleArg = do
    let req = makeRequest "Drawing" "AddDirection" [makeArgument 0 directionArg, makeArgument 1 referenceFrameArg, makeArgument 2 lengthArg, makeArgument 3 visibleArg]
    return (makeStream req)

addDirectionStream :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> Float -> Bool -> RPCContext (KRPCStream (KRPCHS.Drawing.Line))
addDirectionStream directionArg referenceFrameArg lengthArg visibleArg = requestStream =<< addDirectionStreamReq directionArg referenceFrameArg lengthArg visibleArg 

{-
 - Draw a line in the scene.<param name="start">Position of the start of the line.<param name="end">Position of the end of the line.<param name="referenceFrame">Reference frame that the positions are in.<param name="visible">Whether the line is visible.
 -}
addLine :: (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> Bool -> RPCContext (KRPCHS.Drawing.Line)
addLine startArg endArg referenceFrameArg visibleArg = do
    let r = makeRequest "Drawing" "AddLine" [makeArgument 0 startArg, makeArgument 1 endArg, makeArgument 2 referenceFrameArg, makeArgument 3 visibleArg]
    res <- sendRequest r
    processResponse res

addLineStreamReq :: (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> Bool -> RPCContext (KRPCStreamReq (KRPCHS.Drawing.Line))
addLineStreamReq startArg endArg referenceFrameArg visibleArg = do
    let req = makeRequest "Drawing" "AddLine" [makeArgument 0 startArg, makeArgument 1 endArg, makeArgument 2 referenceFrameArg, makeArgument 3 visibleArg]
    return (makeStream req)

addLineStream :: (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> Bool -> RPCContext (KRPCStream (KRPCHS.Drawing.Line))
addLineStream startArg endArg referenceFrameArg visibleArg = requestStream =<< addLineStreamReq startArg endArg referenceFrameArg visibleArg 

{-
 - Draw a polygon in the scene, defined by a list of vertices.<param name="vertices">Vertices of the polygon.<param name="referenceFrame">Reference frame that the vertices are in.<param name="visible">Whether the polygon is visible.
 -}
addPolygon :: [(Double, Double, Double)] -> KRPCHS.SpaceCenter.ReferenceFrame -> Bool -> RPCContext (KRPCHS.Drawing.Polygon)
addPolygon verticesArg referenceFrameArg visibleArg = do
    let r = makeRequest "Drawing" "AddPolygon" [makeArgument 0 verticesArg, makeArgument 1 referenceFrameArg, makeArgument 2 visibleArg]
    res <- sendRequest r
    processResponse res

addPolygonStreamReq :: [(Double, Double, Double)] -> KRPCHS.SpaceCenter.ReferenceFrame -> Bool -> RPCContext (KRPCStreamReq (KRPCHS.Drawing.Polygon))
addPolygonStreamReq verticesArg referenceFrameArg visibleArg = do
    let req = makeRequest "Drawing" "AddPolygon" [makeArgument 0 verticesArg, makeArgument 1 referenceFrameArg, makeArgument 2 visibleArg]
    return (makeStream req)

addPolygonStream :: [(Double, Double, Double)] -> KRPCHS.SpaceCenter.ReferenceFrame -> Bool -> RPCContext (KRPCStream (KRPCHS.Drawing.Polygon))
addPolygonStream verticesArg referenceFrameArg visibleArg = requestStream =<< addPolygonStreamReq verticesArg referenceFrameArg visibleArg 

{-
 - Draw text in the scene.<param name="text">The string to draw.<param name="referenceFrame">Reference frame that the text position is in.<param name="position">Position of the text.<param name="rotation">Rotation of the text, as a quaternion.<param name="visible">Whether the text is visible.
 -}
addText :: Data.Text.Text -> KRPCHS.SpaceCenter.ReferenceFrame -> (Double, Double, Double) -> (Double, Double, Double, Double) -> Bool -> RPCContext (KRPCHS.Drawing.Text)
addText textArg referenceFrameArg positionArg rotationArg visibleArg = do
    let r = makeRequest "Drawing" "AddText" [makeArgument 0 textArg, makeArgument 1 referenceFrameArg, makeArgument 2 positionArg, makeArgument 3 rotationArg, makeArgument 4 visibleArg]
    res <- sendRequest r
    processResponse res

addTextStreamReq :: Data.Text.Text -> KRPCHS.SpaceCenter.ReferenceFrame -> (Double, Double, Double) -> (Double, Double, Double, Double) -> Bool -> RPCContext (KRPCStreamReq (KRPCHS.Drawing.Text))
addTextStreamReq textArg referenceFrameArg positionArg rotationArg visibleArg = do
    let req = makeRequest "Drawing" "AddText" [makeArgument 0 textArg, makeArgument 1 referenceFrameArg, makeArgument 2 positionArg, makeArgument 3 rotationArg, makeArgument 4 visibleArg]
    return (makeStream req)

addTextStream :: Data.Text.Text -> KRPCHS.SpaceCenter.ReferenceFrame -> (Double, Double, Double) -> (Double, Double, Double, Double) -> Bool -> RPCContext (KRPCStream (KRPCHS.Drawing.Text))
addTextStream textArg referenceFrameArg positionArg rotationArg visibleArg = requestStream =<< addTextStreamReq textArg referenceFrameArg positionArg rotationArg visibleArg 

{-
 - Remove all objects being drawn.<param name="clientOnly">If true, only remove objects created by the calling client.
 -}
clear :: Bool -> RPCContext ()
clear clientOnlyArg = do
    let r = makeRequest "Drawing" "Clear" [makeArgument 0 clientOnlyArg]
    res <- sendRequest r
    processResponse res 

{-
 - Remove the object.
 -}
lineRemove :: KRPCHS.Drawing.Line -> RPCContext ()
lineRemove thisArg = do
    let r = makeRequest "Drawing" "Line_Remove" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Set the color
 -}
getLineColor :: KRPCHS.Drawing.Line -> RPCContext ((Double, Double, Double))
getLineColor thisArg = do
    let r = makeRequest "Drawing" "Line_get_Color" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLineColorStreamReq :: KRPCHS.Drawing.Line -> RPCContext (KRPCStreamReq ((Double, Double, Double)))
getLineColorStreamReq thisArg = do
    let req = makeRequest "Drawing" "Line_get_Color" [makeArgument 0 thisArg]
    return (makeStream req)

getLineColorStream :: KRPCHS.Drawing.Line -> RPCContext (KRPCStream ((Double, Double, Double)))
getLineColorStream thisArg = requestStream =<< getLineColorStreamReq thisArg 

{-
 - End position of the line.
 -}
getLineEnd :: KRPCHS.Drawing.Line -> RPCContext ((Double, Double, Double))
getLineEnd thisArg = do
    let r = makeRequest "Drawing" "Line_get_End" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLineEndStreamReq :: KRPCHS.Drawing.Line -> RPCContext (KRPCStreamReq ((Double, Double, Double)))
getLineEndStreamReq thisArg = do
    let req = makeRequest "Drawing" "Line_get_End" [makeArgument 0 thisArg]
    return (makeStream req)

getLineEndStream :: KRPCHS.Drawing.Line -> RPCContext (KRPCStream ((Double, Double, Double)))
getLineEndStream thisArg = requestStream =<< getLineEndStreamReq thisArg 

{-
 - Material used to render the object.
 - Creates the material from a shader with the given name.
 -}
getLineMaterial :: KRPCHS.Drawing.Line -> RPCContext (Data.Text.Text)
getLineMaterial thisArg = do
    let r = makeRequest "Drawing" "Line_get_Material" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLineMaterialStreamReq :: KRPCHS.Drawing.Line -> RPCContext (KRPCStreamReq (Data.Text.Text))
getLineMaterialStreamReq thisArg = do
    let req = makeRequest "Drawing" "Line_get_Material" [makeArgument 0 thisArg]
    return (makeStream req)

getLineMaterialStream :: KRPCHS.Drawing.Line -> RPCContext (KRPCStream (Data.Text.Text))
getLineMaterialStream thisArg = requestStream =<< getLineMaterialStreamReq thisArg 

{-
 - Reference frame for the positions of the object.
 -}
getLineReferenceFrame :: KRPCHS.Drawing.Line -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getLineReferenceFrame thisArg = do
    let r = makeRequest "Drawing" "Line_get_ReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLineReferenceFrameStreamReq :: KRPCHS.Drawing.Line -> RPCContext (KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame))
getLineReferenceFrameStreamReq thisArg = do
    let req = makeRequest "Drawing" "Line_get_ReferenceFrame" [makeArgument 0 thisArg]
    return (makeStream req)

getLineReferenceFrameStream :: KRPCHS.Drawing.Line -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getLineReferenceFrameStream thisArg = requestStream =<< getLineReferenceFrameStreamReq thisArg 

{-
 - Start position of the line.
 -}
getLineStart :: KRPCHS.Drawing.Line -> RPCContext ((Double, Double, Double))
getLineStart thisArg = do
    let r = makeRequest "Drawing" "Line_get_Start" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLineStartStreamReq :: KRPCHS.Drawing.Line -> RPCContext (KRPCStreamReq ((Double, Double, Double)))
getLineStartStreamReq thisArg = do
    let req = makeRequest "Drawing" "Line_get_Start" [makeArgument 0 thisArg]
    return (makeStream req)

getLineStartStream :: KRPCHS.Drawing.Line -> RPCContext (KRPCStream ((Double, Double, Double)))
getLineStartStream thisArg = requestStream =<< getLineStartStreamReq thisArg 

{-
 - Set the thickness
 -}
getLineThickness :: KRPCHS.Drawing.Line -> RPCContext (Float)
getLineThickness thisArg = do
    let r = makeRequest "Drawing" "Line_get_Thickness" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLineThicknessStreamReq :: KRPCHS.Drawing.Line -> RPCContext (KRPCStreamReq (Float))
getLineThicknessStreamReq thisArg = do
    let req = makeRequest "Drawing" "Line_get_Thickness" [makeArgument 0 thisArg]
    return (makeStream req)

getLineThicknessStream :: KRPCHS.Drawing.Line -> RPCContext (KRPCStream (Float))
getLineThicknessStream thisArg = requestStream =<< getLineThicknessStreamReq thisArg 

{-
 - Whether the object is visible.
 -}
getLineVisible :: KRPCHS.Drawing.Line -> RPCContext (Bool)
getLineVisible thisArg = do
    let r = makeRequest "Drawing" "Line_get_Visible" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getLineVisibleStreamReq :: KRPCHS.Drawing.Line -> RPCContext (KRPCStreamReq (Bool))
getLineVisibleStreamReq thisArg = do
    let req = makeRequest "Drawing" "Line_get_Visible" [makeArgument 0 thisArg]
    return (makeStream req)

getLineVisibleStream :: KRPCHS.Drawing.Line -> RPCContext (KRPCStream (Bool))
getLineVisibleStream thisArg = requestStream =<< getLineVisibleStreamReq thisArg 

{-
 - Set the color
 -}
setLineColor :: KRPCHS.Drawing.Line -> (Double, Double, Double) -> RPCContext ()
setLineColor thisArg valueArg = do
    let r = makeRequest "Drawing" "Line_set_Color" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - End position of the line.
 -}
setLineEnd :: KRPCHS.Drawing.Line -> (Double, Double, Double) -> RPCContext ()
setLineEnd thisArg valueArg = do
    let r = makeRequest "Drawing" "Line_set_End" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Material used to render the object.
 - Creates the material from a shader with the given name.
 -}
setLineMaterial :: KRPCHS.Drawing.Line -> Data.Text.Text -> RPCContext ()
setLineMaterial thisArg valueArg = do
    let r = makeRequest "Drawing" "Line_set_Material" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Reference frame for the positions of the object.
 -}
setLineReferenceFrame :: KRPCHS.Drawing.Line -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ()
setLineReferenceFrame thisArg valueArg = do
    let r = makeRequest "Drawing" "Line_set_ReferenceFrame" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Start position of the line.
 -}
setLineStart :: KRPCHS.Drawing.Line -> (Double, Double, Double) -> RPCContext ()
setLineStart thisArg valueArg = do
    let r = makeRequest "Drawing" "Line_set_Start" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Set the thickness
 -}
setLineThickness :: KRPCHS.Drawing.Line -> Float -> RPCContext ()
setLineThickness thisArg valueArg = do
    let r = makeRequest "Drawing" "Line_set_Thickness" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the object is visible.
 -}
setLineVisible :: KRPCHS.Drawing.Line -> Bool -> RPCContext ()
setLineVisible thisArg valueArg = do
    let r = makeRequest "Drawing" "Line_set_Visible" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Remove the object.
 -}
polygonRemove :: KRPCHS.Drawing.Polygon -> RPCContext ()
polygonRemove thisArg = do
    let r = makeRequest "Drawing" "Polygon_Remove" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Set the color
 -}
getPolygonColor :: KRPCHS.Drawing.Polygon -> RPCContext ((Double, Double, Double))
getPolygonColor thisArg = do
    let r = makeRequest "Drawing" "Polygon_get_Color" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPolygonColorStreamReq :: KRPCHS.Drawing.Polygon -> RPCContext (KRPCStreamReq ((Double, Double, Double)))
getPolygonColorStreamReq thisArg = do
    let req = makeRequest "Drawing" "Polygon_get_Color" [makeArgument 0 thisArg]
    return (makeStream req)

getPolygonColorStream :: KRPCHS.Drawing.Polygon -> RPCContext (KRPCStream ((Double, Double, Double)))
getPolygonColorStream thisArg = requestStream =<< getPolygonColorStreamReq thisArg 

{-
 - Material used to render the object.
 - Creates the material from a shader with the given name.
 -}
getPolygonMaterial :: KRPCHS.Drawing.Polygon -> RPCContext (Data.Text.Text)
getPolygonMaterial thisArg = do
    let r = makeRequest "Drawing" "Polygon_get_Material" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPolygonMaterialStreamReq :: KRPCHS.Drawing.Polygon -> RPCContext (KRPCStreamReq (Data.Text.Text))
getPolygonMaterialStreamReq thisArg = do
    let req = makeRequest "Drawing" "Polygon_get_Material" [makeArgument 0 thisArg]
    return (makeStream req)

getPolygonMaterialStream :: KRPCHS.Drawing.Polygon -> RPCContext (KRPCStream (Data.Text.Text))
getPolygonMaterialStream thisArg = requestStream =<< getPolygonMaterialStreamReq thisArg 

{-
 - Reference frame for the positions of the object.
 -}
getPolygonReferenceFrame :: KRPCHS.Drawing.Polygon -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getPolygonReferenceFrame thisArg = do
    let r = makeRequest "Drawing" "Polygon_get_ReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPolygonReferenceFrameStreamReq :: KRPCHS.Drawing.Polygon -> RPCContext (KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame))
getPolygonReferenceFrameStreamReq thisArg = do
    let req = makeRequest "Drawing" "Polygon_get_ReferenceFrame" [makeArgument 0 thisArg]
    return (makeStream req)

getPolygonReferenceFrameStream :: KRPCHS.Drawing.Polygon -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getPolygonReferenceFrameStream thisArg = requestStream =<< getPolygonReferenceFrameStreamReq thisArg 

{-
 - Set the thickness
 -}
getPolygonThickness :: KRPCHS.Drawing.Polygon -> RPCContext (Float)
getPolygonThickness thisArg = do
    let r = makeRequest "Drawing" "Polygon_get_Thickness" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPolygonThicknessStreamReq :: KRPCHS.Drawing.Polygon -> RPCContext (KRPCStreamReq (Float))
getPolygonThicknessStreamReq thisArg = do
    let req = makeRequest "Drawing" "Polygon_get_Thickness" [makeArgument 0 thisArg]
    return (makeStream req)

getPolygonThicknessStream :: KRPCHS.Drawing.Polygon -> RPCContext (KRPCStream (Float))
getPolygonThicknessStream thisArg = requestStream =<< getPolygonThicknessStreamReq thisArg 

{-
 - Vertices for the polygon.
 -}
getPolygonVertices :: KRPCHS.Drawing.Polygon -> RPCContext ([(Double, Double, Double)])
getPolygonVertices thisArg = do
    let r = makeRequest "Drawing" "Polygon_get_Vertices" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPolygonVerticesStreamReq :: KRPCHS.Drawing.Polygon -> RPCContext (KRPCStreamReq ([(Double, Double, Double)]))
getPolygonVerticesStreamReq thisArg = do
    let req = makeRequest "Drawing" "Polygon_get_Vertices" [makeArgument 0 thisArg]
    return (makeStream req)

getPolygonVerticesStream :: KRPCHS.Drawing.Polygon -> RPCContext (KRPCStream ([(Double, Double, Double)]))
getPolygonVerticesStream thisArg = requestStream =<< getPolygonVerticesStreamReq thisArg 

{-
 - Whether the object is visible.
 -}
getPolygonVisible :: KRPCHS.Drawing.Polygon -> RPCContext (Bool)
getPolygonVisible thisArg = do
    let r = makeRequest "Drawing" "Polygon_get_Visible" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPolygonVisibleStreamReq :: KRPCHS.Drawing.Polygon -> RPCContext (KRPCStreamReq (Bool))
getPolygonVisibleStreamReq thisArg = do
    let req = makeRequest "Drawing" "Polygon_get_Visible" [makeArgument 0 thisArg]
    return (makeStream req)

getPolygonVisibleStream :: KRPCHS.Drawing.Polygon -> RPCContext (KRPCStream (Bool))
getPolygonVisibleStream thisArg = requestStream =<< getPolygonVisibleStreamReq thisArg 

{-
 - Set the color
 -}
setPolygonColor :: KRPCHS.Drawing.Polygon -> (Double, Double, Double) -> RPCContext ()
setPolygonColor thisArg valueArg = do
    let r = makeRequest "Drawing" "Polygon_set_Color" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Material used to render the object.
 - Creates the material from a shader with the given name.
 -}
setPolygonMaterial :: KRPCHS.Drawing.Polygon -> Data.Text.Text -> RPCContext ()
setPolygonMaterial thisArg valueArg = do
    let r = makeRequest "Drawing" "Polygon_set_Material" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Reference frame for the positions of the object.
 -}
setPolygonReferenceFrame :: KRPCHS.Drawing.Polygon -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ()
setPolygonReferenceFrame thisArg valueArg = do
    let r = makeRequest "Drawing" "Polygon_set_ReferenceFrame" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Set the thickness
 -}
setPolygonThickness :: KRPCHS.Drawing.Polygon -> Float -> RPCContext ()
setPolygonThickness thisArg valueArg = do
    let r = makeRequest "Drawing" "Polygon_set_Thickness" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Vertices for the polygon.
 -}
setPolygonVertices :: KRPCHS.Drawing.Polygon -> [(Double, Double, Double)] -> RPCContext ()
setPolygonVertices thisArg valueArg = do
    let r = makeRequest "Drawing" "Polygon_set_Vertices" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the object is visible.
 -}
setPolygonVisible :: KRPCHS.Drawing.Polygon -> Bool -> RPCContext ()
setPolygonVisible thisArg valueArg = do
    let r = makeRequest "Drawing" "Polygon_set_Visible" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Remove the object.
 -}
textRemove :: KRPCHS.Drawing.Text -> RPCContext ()
textRemove thisArg = do
    let r = makeRequest "Drawing" "Text_Remove" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Alignment.
 -}
getTextAlignment :: KRPCHS.Drawing.Text -> RPCContext (KRPCHS.UI.TextAlignment)
getTextAlignment thisArg = do
    let r = makeRequest "Drawing" "Text_get_Alignment" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextAlignmentStreamReq :: KRPCHS.Drawing.Text -> RPCContext (KRPCStreamReq (KRPCHS.UI.TextAlignment))
getTextAlignmentStreamReq thisArg = do
    let req = makeRequest "Drawing" "Text_get_Alignment" [makeArgument 0 thisArg]
    return (makeStream req)

getTextAlignmentStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (KRPCHS.UI.TextAlignment))
getTextAlignmentStream thisArg = requestStream =<< getTextAlignmentStreamReq thisArg 

{-
 - Anchor.
 -}
getTextAnchor :: KRPCHS.Drawing.Text -> RPCContext (KRPCHS.UI.TextAnchor)
getTextAnchor thisArg = do
    let r = makeRequest "Drawing" "Text_get_Anchor" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextAnchorStreamReq :: KRPCHS.Drawing.Text -> RPCContext (KRPCStreamReq (KRPCHS.UI.TextAnchor))
getTextAnchorStreamReq thisArg = do
    let req = makeRequest "Drawing" "Text_get_Anchor" [makeArgument 0 thisArg]
    return (makeStream req)

getTextAnchorStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (KRPCHS.UI.TextAnchor))
getTextAnchorStream thisArg = requestStream =<< getTextAnchorStreamReq thisArg 

{-
 - A list of all available fonts.
 -}
getTextAvailableFonts :: KRPCHS.Drawing.Text -> RPCContext ([Data.Text.Text])
getTextAvailableFonts thisArg = do
    let r = makeRequest "Drawing" "Text_get_AvailableFonts" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextAvailableFontsStreamReq :: KRPCHS.Drawing.Text -> RPCContext (KRPCStreamReq ([Data.Text.Text]))
getTextAvailableFontsStreamReq thisArg = do
    let req = makeRequest "Drawing" "Text_get_AvailableFonts" [makeArgument 0 thisArg]
    return (makeStream req)

getTextAvailableFontsStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream ([Data.Text.Text]))
getTextAvailableFontsStream thisArg = requestStream =<< getTextAvailableFontsStreamReq thisArg 

{-
 - Character size.
 -}
getTextCharacterSize :: KRPCHS.Drawing.Text -> RPCContext (Float)
getTextCharacterSize thisArg = do
    let r = makeRequest "Drawing" "Text_get_CharacterSize" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextCharacterSizeStreamReq :: KRPCHS.Drawing.Text -> RPCContext (KRPCStreamReq (Float))
getTextCharacterSizeStreamReq thisArg = do
    let req = makeRequest "Drawing" "Text_get_CharacterSize" [makeArgument 0 thisArg]
    return (makeStream req)

getTextCharacterSizeStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (Float))
getTextCharacterSizeStream thisArg = requestStream =<< getTextCharacterSizeStreamReq thisArg 

{-
 - Set the color
 -}
getTextColor :: KRPCHS.Drawing.Text -> RPCContext ((Double, Double, Double))
getTextColor thisArg = do
    let r = makeRequest "Drawing" "Text_get_Color" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextColorStreamReq :: KRPCHS.Drawing.Text -> RPCContext (KRPCStreamReq ((Double, Double, Double)))
getTextColorStreamReq thisArg = do
    let req = makeRequest "Drawing" "Text_get_Color" [makeArgument 0 thisArg]
    return (makeStream req)

getTextColorStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream ((Double, Double, Double)))
getTextColorStream thisArg = requestStream =<< getTextColorStreamReq thisArg 

{-
 - The text string
 -}
getTextContent :: KRPCHS.Drawing.Text -> RPCContext (Data.Text.Text)
getTextContent thisArg = do
    let r = makeRequest "Drawing" "Text_get_Content" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextContentStreamReq :: KRPCHS.Drawing.Text -> RPCContext (KRPCStreamReq (Data.Text.Text))
getTextContentStreamReq thisArg = do
    let req = makeRequest "Drawing" "Text_get_Content" [makeArgument 0 thisArg]
    return (makeStream req)

getTextContentStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (Data.Text.Text))
getTextContentStream thisArg = requestStream =<< getTextContentStreamReq thisArg 

{-
 - Name of the font
 -}
getTextFont :: KRPCHS.Drawing.Text -> RPCContext (Data.Text.Text)
getTextFont thisArg = do
    let r = makeRequest "Drawing" "Text_get_Font" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextFontStreamReq :: KRPCHS.Drawing.Text -> RPCContext (KRPCStreamReq (Data.Text.Text))
getTextFontStreamReq thisArg = do
    let req = makeRequest "Drawing" "Text_get_Font" [makeArgument 0 thisArg]
    return (makeStream req)

getTextFontStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (Data.Text.Text))
getTextFontStream thisArg = requestStream =<< getTextFontStreamReq thisArg 

{-
 - Line spacing.
 -}
getTextLineSpacing :: KRPCHS.Drawing.Text -> RPCContext (Float)
getTextLineSpacing thisArg = do
    let r = makeRequest "Drawing" "Text_get_LineSpacing" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextLineSpacingStreamReq :: KRPCHS.Drawing.Text -> RPCContext (KRPCStreamReq (Float))
getTextLineSpacingStreamReq thisArg = do
    let req = makeRequest "Drawing" "Text_get_LineSpacing" [makeArgument 0 thisArg]
    return (makeStream req)

getTextLineSpacingStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (Float))
getTextLineSpacingStream thisArg = requestStream =<< getTextLineSpacingStreamReq thisArg 

{-
 - Material used to render the object.
 - Creates the material from a shader with the given name.
 -}
getTextMaterial :: KRPCHS.Drawing.Text -> RPCContext (Data.Text.Text)
getTextMaterial thisArg = do
    let r = makeRequest "Drawing" "Text_get_Material" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextMaterialStreamReq :: KRPCHS.Drawing.Text -> RPCContext (KRPCStreamReq (Data.Text.Text))
getTextMaterialStreamReq thisArg = do
    let req = makeRequest "Drawing" "Text_get_Material" [makeArgument 0 thisArg]
    return (makeStream req)

getTextMaterialStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (Data.Text.Text))
getTextMaterialStream thisArg = requestStream =<< getTextMaterialStreamReq thisArg 

{-
 - Position of the text.
 -}
getTextPosition :: KRPCHS.Drawing.Text -> RPCContext ((Double, Double, Double))
getTextPosition thisArg = do
    let r = makeRequest "Drawing" "Text_get_Position" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextPositionStreamReq :: KRPCHS.Drawing.Text -> RPCContext (KRPCStreamReq ((Double, Double, Double)))
getTextPositionStreamReq thisArg = do
    let req = makeRequest "Drawing" "Text_get_Position" [makeArgument 0 thisArg]
    return (makeStream req)

getTextPositionStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream ((Double, Double, Double)))
getTextPositionStream thisArg = requestStream =<< getTextPositionStreamReq thisArg 

{-
 - Reference frame for the positions of the object.
 -}
getTextReferenceFrame :: KRPCHS.Drawing.Text -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getTextReferenceFrame thisArg = do
    let r = makeRequest "Drawing" "Text_get_ReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextReferenceFrameStreamReq :: KRPCHS.Drawing.Text -> RPCContext (KRPCStreamReq (KRPCHS.SpaceCenter.ReferenceFrame))
getTextReferenceFrameStreamReq thisArg = do
    let req = makeRequest "Drawing" "Text_get_ReferenceFrame" [makeArgument 0 thisArg]
    return (makeStream req)

getTextReferenceFrameStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getTextReferenceFrameStream thisArg = requestStream =<< getTextReferenceFrameStreamReq thisArg 

{-
 - Rotation of the text as a quaternion.
 -}
getTextRotation :: KRPCHS.Drawing.Text -> RPCContext ((Double, Double, Double, Double))
getTextRotation thisArg = do
    let r = makeRequest "Drawing" "Text_get_Rotation" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextRotationStreamReq :: KRPCHS.Drawing.Text -> RPCContext (KRPCStreamReq ((Double, Double, Double, Double)))
getTextRotationStreamReq thisArg = do
    let req = makeRequest "Drawing" "Text_get_Rotation" [makeArgument 0 thisArg]
    return (makeStream req)

getTextRotationStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
getTextRotationStream thisArg = requestStream =<< getTextRotationStreamReq thisArg 

{-
 - Font size.
 -}
getTextSize :: KRPCHS.Drawing.Text -> RPCContext (Data.Int.Int32)
getTextSize thisArg = do
    let r = makeRequest "Drawing" "Text_get_Size" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextSizeStreamReq :: KRPCHS.Drawing.Text -> RPCContext (KRPCStreamReq (Data.Int.Int32))
getTextSizeStreamReq thisArg = do
    let req = makeRequest "Drawing" "Text_get_Size" [makeArgument 0 thisArg]
    return (makeStream req)

getTextSizeStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (Data.Int.Int32))
getTextSizeStream thisArg = requestStream =<< getTextSizeStreamReq thisArg 

{-
 - Font style.
 -}
getTextStyle :: KRPCHS.Drawing.Text -> RPCContext (KRPCHS.UI.FontStyle)
getTextStyle thisArg = do
    let r = makeRequest "Drawing" "Text_get_Style" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextStyleStreamReq :: KRPCHS.Drawing.Text -> RPCContext (KRPCStreamReq (KRPCHS.UI.FontStyle))
getTextStyleStreamReq thisArg = do
    let req = makeRequest "Drawing" "Text_get_Style" [makeArgument 0 thisArg]
    return (makeStream req)

getTextStyleStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (KRPCHS.UI.FontStyle))
getTextStyleStream thisArg = requestStream =<< getTextStyleStreamReq thisArg 

{-
 - Whether the object is visible.
 -}
getTextVisible :: KRPCHS.Drawing.Text -> RPCContext (Bool)
getTextVisible thisArg = do
    let r = makeRequest "Drawing" "Text_get_Visible" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextVisibleStreamReq :: KRPCHS.Drawing.Text -> RPCContext (KRPCStreamReq (Bool))
getTextVisibleStreamReq thisArg = do
    let req = makeRequest "Drawing" "Text_get_Visible" [makeArgument 0 thisArg]
    return (makeStream req)

getTextVisibleStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (Bool))
getTextVisibleStream thisArg = requestStream =<< getTextVisibleStreamReq thisArg 

{-
 - Alignment.
 -}
setTextAlignment :: KRPCHS.Drawing.Text -> KRPCHS.UI.TextAlignment -> RPCContext ()
setTextAlignment thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Alignment" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Anchor.
 -}
setTextAnchor :: KRPCHS.Drawing.Text -> KRPCHS.UI.TextAnchor -> RPCContext ()
setTextAnchor thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Anchor" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Character size.
 -}
setTextCharacterSize :: KRPCHS.Drawing.Text -> Float -> RPCContext ()
setTextCharacterSize thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_CharacterSize" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Set the color
 -}
setTextColor :: KRPCHS.Drawing.Text -> (Double, Double, Double) -> RPCContext ()
setTextColor thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Color" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The text string
 -}
setTextContent :: KRPCHS.Drawing.Text -> Data.Text.Text -> RPCContext ()
setTextContent thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Content" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Name of the font
 -}
setTextFont :: KRPCHS.Drawing.Text -> Data.Text.Text -> RPCContext ()
setTextFont thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Font" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Line spacing.
 -}
setTextLineSpacing :: KRPCHS.Drawing.Text -> Float -> RPCContext ()
setTextLineSpacing thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_LineSpacing" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Material used to render the object.
 - Creates the material from a shader with the given name.
 -}
setTextMaterial :: KRPCHS.Drawing.Text -> Data.Text.Text -> RPCContext ()
setTextMaterial thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Material" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Position of the text.
 -}
setTextPosition :: KRPCHS.Drawing.Text -> (Double, Double, Double) -> RPCContext ()
setTextPosition thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Position" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Reference frame for the positions of the object.
 -}
setTextReferenceFrame :: KRPCHS.Drawing.Text -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext ()
setTextReferenceFrame thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_ReferenceFrame" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Rotation of the text as a quaternion.
 -}
setTextRotation :: KRPCHS.Drawing.Text -> (Double, Double, Double, Double) -> RPCContext ()
setTextRotation thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Rotation" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Font size.
 -}
setTextSize :: KRPCHS.Drawing.Text -> Data.Int.Int32 -> RPCContext ()
setTextSize thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Size" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Font style.
 -}
setTextStyle :: KRPCHS.Drawing.Text -> KRPCHS.UI.FontStyle -> RPCContext ()
setTextStyle thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Style" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the object is visible.
 -}
setTextVisible :: KRPCHS.Drawing.Text -> Bool -> RPCContext ()
setTextVisible thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Visible" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

