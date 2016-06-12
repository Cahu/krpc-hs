module KRPCHS.Drawing
( Line
, Polygon
, Text
, addDirection
, addDirectionStream
, addLine
, addLineStream
, addPolygon
, addPolygonStream
, addText
, addTextStream
, clear
, lineRemove
, getLineColor
, getLineColorStream
, getLineEnd
, getLineEndStream
, getLineMaterial
, getLineMaterialStream
, getLineReferenceFrame
, getLineReferenceFrameStream
, getLineStart
, getLineStartStream
, getLineThickness
, getLineThicknessStream
, getLineVisible
, getLineVisibleStream
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
, getPolygonMaterial
, getPolygonMaterialStream
, getPolygonReferenceFrame
, getPolygonReferenceFrameStream
, getPolygonThickness
, getPolygonThicknessStream
, getPolygonVertices
, getPolygonVerticesStream
, getPolygonVisible
, getPolygonVisibleStream
, setPolygonColor
, setPolygonMaterial
, setPolygonReferenceFrame
, setPolygonThickness
, setPolygonVertices
, setPolygonVisible
, textRemove
, getTextAlignment
, getTextAlignmentStream
, getTextAnchor
, getTextAnchorStream
, getTextAvailableFonts
, getTextAvailableFontsStream
, getTextCharacterSize
, getTextCharacterSizeStream
, getTextColor
, getTextColorStream
, getTextContent
, getTextContentStream
, getTextFont
, getTextFontStream
, getTextLineSpacing
, getTextLineSpacingStream
, getTextMaterial
, getTextMaterialStream
, getTextPosition
, getTextPositionStream
, getTextReferenceFrame
, getTextReferenceFrameStream
, getTextRotation
, getTextRotationStream
, getTextSize
, getTextSizeStream
, getTextStyle
, getTextStyleStream
, getTextVisible
, getTextVisibleStream
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
    processResponse extract res 

addDirectionStream :: (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> Float -> Bool -> RPCContext (KRPCStream (KRPCHS.Drawing.Line))
addDirectionStream directionArg referenceFrameArg lengthArg visibleArg = do
    let r = makeRequest "Drawing" "AddDirection" [makeArgument 0 directionArg, makeArgument 1 referenceFrameArg, makeArgument 2 lengthArg, makeArgument 3 visibleArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Draw a line in the scene.<param name="start">Position of the start of the line.<param name="end">Position of the end of the line.<param name="referenceFrame">Reference frame that the positions are in.<param name="visible">Whether the line is visible.
 -}
addLine :: (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> Bool -> RPCContext (KRPCHS.Drawing.Line)
addLine startArg endArg referenceFrameArg visibleArg = do
    let r = makeRequest "Drawing" "AddLine" [makeArgument 0 startArg, makeArgument 1 endArg, makeArgument 2 referenceFrameArg, makeArgument 3 visibleArg]
    res <- sendRequest r
    processResponse extract res 

addLineStream :: (Double, Double, Double) -> (Double, Double, Double) -> KRPCHS.SpaceCenter.ReferenceFrame -> Bool -> RPCContext (KRPCStream (KRPCHS.Drawing.Line))
addLineStream startArg endArg referenceFrameArg visibleArg = do
    let r = makeRequest "Drawing" "AddLine" [makeArgument 0 startArg, makeArgument 1 endArg, makeArgument 2 referenceFrameArg, makeArgument 3 visibleArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Draw a polygon in the scene, defined by a list of vertices.<param name="vertices">Vertices of the polygon.<param name="referenceFrame">Reference frame that the vertices are in.<param name="visible">Whether the polygon is visible.
 -}
addPolygon :: [(Double, Double, Double)] -> KRPCHS.SpaceCenter.ReferenceFrame -> Bool -> RPCContext (KRPCHS.Drawing.Polygon)
addPolygon verticesArg referenceFrameArg visibleArg = do
    let r = makeRequest "Drawing" "AddPolygon" [makeArgument 0 verticesArg, makeArgument 1 referenceFrameArg, makeArgument 2 visibleArg]
    res <- sendRequest r
    processResponse extract res 

addPolygonStream :: [(Double, Double, Double)] -> KRPCHS.SpaceCenter.ReferenceFrame -> Bool -> RPCContext (KRPCStream (KRPCHS.Drawing.Polygon))
addPolygonStream verticesArg referenceFrameArg visibleArg = do
    let r = makeRequest "Drawing" "AddPolygon" [makeArgument 0 verticesArg, makeArgument 1 referenceFrameArg, makeArgument 2 visibleArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Draw text in the scene.<param name="text">The string to draw.<param name="referenceFrame">Reference frame that the text position is in.<param name="position">Position of the text.<param name="rotation">Rotation of the text, as a quaternion.<param name="visible">Whether the text is visible.
 -}
addText :: Data.Text.Text -> KRPCHS.SpaceCenter.ReferenceFrame -> (Double, Double, Double) -> (Double, Double, Double, Double) -> Bool -> RPCContext (KRPCHS.Drawing.Text)
addText textArg referenceFrameArg positionArg rotationArg visibleArg = do
    let r = makeRequest "Drawing" "AddText" [makeArgument 0 textArg, makeArgument 1 referenceFrameArg, makeArgument 2 positionArg, makeArgument 3 rotationArg, makeArgument 4 visibleArg]
    res <- sendRequest r
    processResponse extract res 

addTextStream :: Data.Text.Text -> KRPCHS.SpaceCenter.ReferenceFrame -> (Double, Double, Double) -> (Double, Double, Double, Double) -> Bool -> RPCContext (KRPCStream (KRPCHS.Drawing.Text))
addTextStream textArg referenceFrameArg positionArg rotationArg visibleArg = do
    let r = makeRequest "Drawing" "AddText" [makeArgument 0 textArg, makeArgument 1 referenceFrameArg, makeArgument 2 positionArg, makeArgument 3 rotationArg, makeArgument 4 visibleArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Remove all objects being drawn.<param name="clientOnly">If true, only remove objects created by the calling client.
 -}
clear :: Bool -> RPCContext (Bool)
clear clientOnlyArg = do
    let r = makeRequest "Drawing" "Clear" [makeArgument 0 clientOnlyArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Remove the object.
 -}
lineRemove :: KRPCHS.Drawing.Line -> RPCContext (Bool)
lineRemove thisArg = do
    let r = makeRequest "Drawing" "Line_Remove" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Set the color
 -}
getLineColor :: KRPCHS.Drawing.Line -> RPCContext ((Double, Double, Double))
getLineColor thisArg = do
    let r = makeRequest "Drawing" "Line_get_Color" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getLineColorStream :: KRPCHS.Drawing.Line -> RPCContext (KRPCStream ((Double, Double, Double)))
getLineColorStream thisArg = do
    let r = makeRequest "Drawing" "Line_get_Color" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - End position of the line.
 -}
getLineEnd :: KRPCHS.Drawing.Line -> RPCContext ((Double, Double, Double))
getLineEnd thisArg = do
    let r = makeRequest "Drawing" "Line_get_End" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getLineEndStream :: KRPCHS.Drawing.Line -> RPCContext (KRPCStream ((Double, Double, Double)))
getLineEndStream thisArg = do
    let r = makeRequest "Drawing" "Line_get_End" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Material used to render the object.
 - Creates the material from a shader with the given name.
 -}
getLineMaterial :: KRPCHS.Drawing.Line -> RPCContext (Data.Text.Text)
getLineMaterial thisArg = do
    let r = makeRequest "Drawing" "Line_get_Material" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getLineMaterialStream :: KRPCHS.Drawing.Line -> RPCContext (KRPCStream (Data.Text.Text))
getLineMaterialStream thisArg = do
    let r = makeRequest "Drawing" "Line_get_Material" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Reference frame for the positions of the object.
 -}
getLineReferenceFrame :: KRPCHS.Drawing.Line -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getLineReferenceFrame thisArg = do
    let r = makeRequest "Drawing" "Line_get_ReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getLineReferenceFrameStream :: KRPCHS.Drawing.Line -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getLineReferenceFrameStream thisArg = do
    let r = makeRequest "Drawing" "Line_get_ReferenceFrame" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Start position of the line.
 -}
getLineStart :: KRPCHS.Drawing.Line -> RPCContext ((Double, Double, Double))
getLineStart thisArg = do
    let r = makeRequest "Drawing" "Line_get_Start" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getLineStartStream :: KRPCHS.Drawing.Line -> RPCContext (KRPCStream ((Double, Double, Double)))
getLineStartStream thisArg = do
    let r = makeRequest "Drawing" "Line_get_Start" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Set the thickness
 -}
getLineThickness :: KRPCHS.Drawing.Line -> RPCContext (Float)
getLineThickness thisArg = do
    let r = makeRequest "Drawing" "Line_get_Thickness" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getLineThicknessStream :: KRPCHS.Drawing.Line -> RPCContext (KRPCStream (Float))
getLineThicknessStream thisArg = do
    let r = makeRequest "Drawing" "Line_get_Thickness" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the object is visible.
 -}
getLineVisible :: KRPCHS.Drawing.Line -> RPCContext (Bool)
getLineVisible thisArg = do
    let r = makeRequest "Drawing" "Line_get_Visible" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getLineVisibleStream :: KRPCHS.Drawing.Line -> RPCContext (KRPCStream (Bool))
getLineVisibleStream thisArg = do
    let r = makeRequest "Drawing" "Line_get_Visible" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Set the color
 -}
setLineColor :: KRPCHS.Drawing.Line -> (Double, Double, Double) -> RPCContext (Bool)
setLineColor thisArg valueArg = do
    let r = makeRequest "Drawing" "Line_set_Color" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - End position of the line.
 -}
setLineEnd :: KRPCHS.Drawing.Line -> (Double, Double, Double) -> RPCContext (Bool)
setLineEnd thisArg valueArg = do
    let r = makeRequest "Drawing" "Line_set_End" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Material used to render the object.
 - Creates the material from a shader with the given name.
 -}
setLineMaterial :: KRPCHS.Drawing.Line -> Data.Text.Text -> RPCContext (Bool)
setLineMaterial thisArg valueArg = do
    let r = makeRequest "Drawing" "Line_set_Material" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Reference frame for the positions of the object.
 -}
setLineReferenceFrame :: KRPCHS.Drawing.Line -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (Bool)
setLineReferenceFrame thisArg valueArg = do
    let r = makeRequest "Drawing" "Line_set_ReferenceFrame" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Start position of the line.
 -}
setLineStart :: KRPCHS.Drawing.Line -> (Double, Double, Double) -> RPCContext (Bool)
setLineStart thisArg valueArg = do
    let r = makeRequest "Drawing" "Line_set_Start" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Set the thickness
 -}
setLineThickness :: KRPCHS.Drawing.Line -> Float -> RPCContext (Bool)
setLineThickness thisArg valueArg = do
    let r = makeRequest "Drawing" "Line_set_Thickness" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the object is visible.
 -}
setLineVisible :: KRPCHS.Drawing.Line -> Bool -> RPCContext (Bool)
setLineVisible thisArg valueArg = do
    let r = makeRequest "Drawing" "Line_set_Visible" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Remove the object.
 -}
polygonRemove :: KRPCHS.Drawing.Polygon -> RPCContext (Bool)
polygonRemove thisArg = do
    let r = makeRequest "Drawing" "Polygon_Remove" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Set the color
 -}
getPolygonColor :: KRPCHS.Drawing.Polygon -> RPCContext ((Double, Double, Double))
getPolygonColor thisArg = do
    let r = makeRequest "Drawing" "Polygon_get_Color" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPolygonColorStream :: KRPCHS.Drawing.Polygon -> RPCContext (KRPCStream ((Double, Double, Double)))
getPolygonColorStream thisArg = do
    let r = makeRequest "Drawing" "Polygon_get_Color" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Material used to render the object.
 - Creates the material from a shader with the given name.
 -}
getPolygonMaterial :: KRPCHS.Drawing.Polygon -> RPCContext (Data.Text.Text)
getPolygonMaterial thisArg = do
    let r = makeRequest "Drawing" "Polygon_get_Material" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPolygonMaterialStream :: KRPCHS.Drawing.Polygon -> RPCContext (KRPCStream (Data.Text.Text))
getPolygonMaterialStream thisArg = do
    let r = makeRequest "Drawing" "Polygon_get_Material" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Reference frame for the positions of the object.
 -}
getPolygonReferenceFrame :: KRPCHS.Drawing.Polygon -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getPolygonReferenceFrame thisArg = do
    let r = makeRequest "Drawing" "Polygon_get_ReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPolygonReferenceFrameStream :: KRPCHS.Drawing.Polygon -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getPolygonReferenceFrameStream thisArg = do
    let r = makeRequest "Drawing" "Polygon_get_ReferenceFrame" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Set the thickness
 -}
getPolygonThickness :: KRPCHS.Drawing.Polygon -> RPCContext (Float)
getPolygonThickness thisArg = do
    let r = makeRequest "Drawing" "Polygon_get_Thickness" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPolygonThicknessStream :: KRPCHS.Drawing.Polygon -> RPCContext (KRPCStream (Float))
getPolygonThicknessStream thisArg = do
    let r = makeRequest "Drawing" "Polygon_get_Thickness" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Vertices for the polygon.
 -}
getPolygonVertices :: KRPCHS.Drawing.Polygon -> RPCContext ([(Double, Double, Double)])
getPolygonVertices thisArg = do
    let r = makeRequest "Drawing" "Polygon_get_Vertices" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPolygonVerticesStream :: KRPCHS.Drawing.Polygon -> RPCContext (KRPCStream ([(Double, Double, Double)]))
getPolygonVerticesStream thisArg = do
    let r = makeRequest "Drawing" "Polygon_get_Vertices" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the object is visible.
 -}
getPolygonVisible :: KRPCHS.Drawing.Polygon -> RPCContext (Bool)
getPolygonVisible thisArg = do
    let r = makeRequest "Drawing" "Polygon_get_Visible" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPolygonVisibleStream :: KRPCHS.Drawing.Polygon -> RPCContext (KRPCStream (Bool))
getPolygonVisibleStream thisArg = do
    let r = makeRequest "Drawing" "Polygon_get_Visible" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Set the color
 -}
setPolygonColor :: KRPCHS.Drawing.Polygon -> (Double, Double, Double) -> RPCContext (Bool)
setPolygonColor thisArg valueArg = do
    let r = makeRequest "Drawing" "Polygon_set_Color" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Material used to render the object.
 - Creates the material from a shader with the given name.
 -}
setPolygonMaterial :: KRPCHS.Drawing.Polygon -> Data.Text.Text -> RPCContext (Bool)
setPolygonMaterial thisArg valueArg = do
    let r = makeRequest "Drawing" "Polygon_set_Material" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Reference frame for the positions of the object.
 -}
setPolygonReferenceFrame :: KRPCHS.Drawing.Polygon -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (Bool)
setPolygonReferenceFrame thisArg valueArg = do
    let r = makeRequest "Drawing" "Polygon_set_ReferenceFrame" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Set the thickness
 -}
setPolygonThickness :: KRPCHS.Drawing.Polygon -> Float -> RPCContext (Bool)
setPolygonThickness thisArg valueArg = do
    let r = makeRequest "Drawing" "Polygon_set_Thickness" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Vertices for the polygon.
 -}
setPolygonVertices :: KRPCHS.Drawing.Polygon -> [(Double, Double, Double)] -> RPCContext (Bool)
setPolygonVertices thisArg valueArg = do
    let r = makeRequest "Drawing" "Polygon_set_Vertices" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the object is visible.
 -}
setPolygonVisible :: KRPCHS.Drawing.Polygon -> Bool -> RPCContext (Bool)
setPolygonVisible thisArg valueArg = do
    let r = makeRequest "Drawing" "Polygon_set_Visible" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Remove the object.
 -}
textRemove :: KRPCHS.Drawing.Text -> RPCContext (Bool)
textRemove thisArg = do
    let r = makeRequest "Drawing" "Text_Remove" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Alignment.
 -}
getTextAlignment :: KRPCHS.Drawing.Text -> RPCContext (KRPCHS.UI.TextAlignment)
getTextAlignment thisArg = do
    let r = makeRequest "Drawing" "Text_get_Alignment" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getTextAlignmentStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (KRPCHS.UI.TextAlignment))
getTextAlignmentStream thisArg = do
    let r = makeRequest "Drawing" "Text_get_Alignment" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Anchor.
 -}
getTextAnchor :: KRPCHS.Drawing.Text -> RPCContext (KRPCHS.UI.TextAnchor)
getTextAnchor thisArg = do
    let r = makeRequest "Drawing" "Text_get_Anchor" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getTextAnchorStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (KRPCHS.UI.TextAnchor))
getTextAnchorStream thisArg = do
    let r = makeRequest "Drawing" "Text_get_Anchor" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of all available fonts.
 -}
getTextAvailableFonts :: KRPCHS.Drawing.Text -> RPCContext ([Data.Text.Text])
getTextAvailableFonts thisArg = do
    let r = makeRequest "Drawing" "Text_get_AvailableFonts" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getTextAvailableFontsStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream ([Data.Text.Text]))
getTextAvailableFontsStream thisArg = do
    let r = makeRequest "Drawing" "Text_get_AvailableFonts" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Character size.
 -}
getTextCharacterSize :: KRPCHS.Drawing.Text -> RPCContext (Float)
getTextCharacterSize thisArg = do
    let r = makeRequest "Drawing" "Text_get_CharacterSize" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getTextCharacterSizeStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (Float))
getTextCharacterSizeStream thisArg = do
    let r = makeRequest "Drawing" "Text_get_CharacterSize" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Set the color
 -}
getTextColor :: KRPCHS.Drawing.Text -> RPCContext ((Double, Double, Double))
getTextColor thisArg = do
    let r = makeRequest "Drawing" "Text_get_Color" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getTextColorStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream ((Double, Double, Double)))
getTextColorStream thisArg = do
    let r = makeRequest "Drawing" "Text_get_Color" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The text string
 -}
getTextContent :: KRPCHS.Drawing.Text -> RPCContext (Data.Text.Text)
getTextContent thisArg = do
    let r = makeRequest "Drawing" "Text_get_Content" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getTextContentStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (Data.Text.Text))
getTextContentStream thisArg = do
    let r = makeRequest "Drawing" "Text_get_Content" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Name of the font
 -}
getTextFont :: KRPCHS.Drawing.Text -> RPCContext (Data.Text.Text)
getTextFont thisArg = do
    let r = makeRequest "Drawing" "Text_get_Font" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getTextFontStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (Data.Text.Text))
getTextFontStream thisArg = do
    let r = makeRequest "Drawing" "Text_get_Font" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Line spacing.
 -}
getTextLineSpacing :: KRPCHS.Drawing.Text -> RPCContext (Float)
getTextLineSpacing thisArg = do
    let r = makeRequest "Drawing" "Text_get_LineSpacing" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getTextLineSpacingStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (Float))
getTextLineSpacingStream thisArg = do
    let r = makeRequest "Drawing" "Text_get_LineSpacing" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Material used to render the object.
 - Creates the material from a shader with the given name.
 -}
getTextMaterial :: KRPCHS.Drawing.Text -> RPCContext (Data.Text.Text)
getTextMaterial thisArg = do
    let r = makeRequest "Drawing" "Text_get_Material" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getTextMaterialStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (Data.Text.Text))
getTextMaterialStream thisArg = do
    let r = makeRequest "Drawing" "Text_get_Material" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Position of the text.
 -}
getTextPosition :: KRPCHS.Drawing.Text -> RPCContext ((Double, Double, Double))
getTextPosition thisArg = do
    let r = makeRequest "Drawing" "Text_get_Position" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getTextPositionStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream ((Double, Double, Double)))
getTextPositionStream thisArg = do
    let r = makeRequest "Drawing" "Text_get_Position" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Reference frame for the positions of the object.
 -}
getTextReferenceFrame :: KRPCHS.Drawing.Text -> RPCContext (KRPCHS.SpaceCenter.ReferenceFrame)
getTextReferenceFrame thisArg = do
    let r = makeRequest "Drawing" "Text_get_ReferenceFrame" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getTextReferenceFrameStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (KRPCHS.SpaceCenter.ReferenceFrame))
getTextReferenceFrameStream thisArg = do
    let r = makeRequest "Drawing" "Text_get_ReferenceFrame" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Rotation of the text as a quaternion.
 -}
getTextRotation :: KRPCHS.Drawing.Text -> RPCContext ((Double, Double, Double, Double))
getTextRotation thisArg = do
    let r = makeRequest "Drawing" "Text_get_Rotation" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getTextRotationStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
getTextRotationStream thisArg = do
    let r = makeRequest "Drawing" "Text_get_Rotation" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Font size.
 -}
getTextSize :: KRPCHS.Drawing.Text -> RPCContext (Data.Int.Int32)
getTextSize thisArg = do
    let r = makeRequest "Drawing" "Text_get_Size" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getTextSizeStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (Data.Int.Int32))
getTextSizeStream thisArg = do
    let r = makeRequest "Drawing" "Text_get_Size" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Font style.
 -}
getTextStyle :: KRPCHS.Drawing.Text -> RPCContext (KRPCHS.UI.FontStyle)
getTextStyle thisArg = do
    let r = makeRequest "Drawing" "Text_get_Style" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getTextStyleStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (KRPCHS.UI.FontStyle))
getTextStyleStream thisArg = do
    let r = makeRequest "Drawing" "Text_get_Style" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the object is visible.
 -}
getTextVisible :: KRPCHS.Drawing.Text -> RPCContext (Bool)
getTextVisible thisArg = do
    let r = makeRequest "Drawing" "Text_get_Visible" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getTextVisibleStream :: KRPCHS.Drawing.Text -> RPCContext (KRPCStream (Bool))
getTextVisibleStream thisArg = do
    let r = makeRequest "Drawing" "Text_get_Visible" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Alignment.
 -}
setTextAlignment :: KRPCHS.Drawing.Text -> KRPCHS.UI.TextAlignment -> RPCContext (Bool)
setTextAlignment thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Alignment" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Anchor.
 -}
setTextAnchor :: KRPCHS.Drawing.Text -> KRPCHS.UI.TextAnchor -> RPCContext (Bool)
setTextAnchor thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Anchor" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Character size.
 -}
setTextCharacterSize :: KRPCHS.Drawing.Text -> Float -> RPCContext (Bool)
setTextCharacterSize thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_CharacterSize" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Set the color
 -}
setTextColor :: KRPCHS.Drawing.Text -> (Double, Double, Double) -> RPCContext (Bool)
setTextColor thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Color" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The text string
 -}
setTextContent :: KRPCHS.Drawing.Text -> Data.Text.Text -> RPCContext (Bool)
setTextContent thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Content" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Name of the font
 -}
setTextFont :: KRPCHS.Drawing.Text -> Data.Text.Text -> RPCContext (Bool)
setTextFont thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Font" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Line spacing.
 -}
setTextLineSpacing :: KRPCHS.Drawing.Text -> Float -> RPCContext (Bool)
setTextLineSpacing thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_LineSpacing" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Material used to render the object.
 - Creates the material from a shader with the given name.
 -}
setTextMaterial :: KRPCHS.Drawing.Text -> Data.Text.Text -> RPCContext (Bool)
setTextMaterial thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Material" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Position of the text.
 -}
setTextPosition :: KRPCHS.Drawing.Text -> (Double, Double, Double) -> RPCContext (Bool)
setTextPosition thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Position" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Reference frame for the positions of the object.
 -}
setTextReferenceFrame :: KRPCHS.Drawing.Text -> KRPCHS.SpaceCenter.ReferenceFrame -> RPCContext (Bool)
setTextReferenceFrame thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_ReferenceFrame" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Rotation of the text as a quaternion.
 -}
setTextRotation :: KRPCHS.Drawing.Text -> (Double, Double, Double, Double) -> RPCContext (Bool)
setTextRotation thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Rotation" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Font size.
 -}
setTextSize :: KRPCHS.Drawing.Text -> Data.Int.Int32 -> RPCContext (Bool)
setTextSize thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Size" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Font style.
 -}
setTextStyle :: KRPCHS.Drawing.Text -> KRPCHS.UI.FontStyle -> RPCContext (Bool)
setTextStyle thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Style" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the object is visible.
 -}
setTextVisible :: KRPCHS.Drawing.Text -> Bool -> RPCContext (Bool)
setTextVisible thisArg valueArg = do
    let r = makeRequest "Drawing" "Text_set_Visible" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


