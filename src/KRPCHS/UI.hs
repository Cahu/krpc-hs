module KRPCHS.UI
( FontStyle(..)
, MessagePosition(..)
, TextAlignment(..)
, TextAnchor(..)
, Button
, Canvas
, InputField
, Panel
, RectTransform
, Text
, addCanvas
, addCanvasReq
, addCanvasStream
, addCanvasStreamReq
, buttonRemove
, buttonRemoveReq
, getButtonClicked
, getButtonClickedReq
, getButtonClickedStream
, getButtonClickedStreamReq
, getButtonRectTransform
, getButtonRectTransformReq
, getButtonRectTransformStream
, getButtonRectTransformStreamReq
, getButtonText
, getButtonTextReq
, getButtonTextStream
, getButtonTextStreamReq
, getButtonVisible
, getButtonVisibleReq
, getButtonVisibleStream
, getButtonVisibleStreamReq
, setButtonClicked
, setButtonClickedReq
, setButtonVisible
, setButtonVisibleReq
, canvasAddButton
, canvasAddButtonReq
, canvasAddButtonStream
, canvasAddButtonStreamReq
, canvasAddInputField
, canvasAddInputFieldReq
, canvasAddInputFieldStream
, canvasAddInputFieldStreamReq
, canvasAddPanel
, canvasAddPanelReq
, canvasAddPanelStream
, canvasAddPanelStreamReq
, canvasAddText
, canvasAddTextReq
, canvasAddTextStream
, canvasAddTextStreamReq
, canvasRemove
, canvasRemoveReq
, getCanvasRectTransform
, getCanvasRectTransformReq
, getCanvasRectTransformStream
, getCanvasRectTransformStreamReq
, getCanvasVisible
, getCanvasVisibleReq
, getCanvasVisibleStream
, getCanvasVisibleStreamReq
, setCanvasVisible
, setCanvasVisibleReq
, clear
, clearReq
, inputFieldRemove
, inputFieldRemoveReq
, getInputFieldChanged
, getInputFieldChangedReq
, getInputFieldChangedStream
, getInputFieldChangedStreamReq
, getInputFieldRectTransform
, getInputFieldRectTransformReq
, getInputFieldRectTransformStream
, getInputFieldRectTransformStreamReq
, getInputFieldText
, getInputFieldTextReq
, getInputFieldTextStream
, getInputFieldTextStreamReq
, getInputFieldValue
, getInputFieldValueReq
, getInputFieldValueStream
, getInputFieldValueStreamReq
, getInputFieldVisible
, getInputFieldVisibleReq
, getInputFieldVisibleStream
, getInputFieldVisibleStreamReq
, setInputFieldChanged
, setInputFieldChangedReq
, setInputFieldValue
, setInputFieldValueReq
, setInputFieldVisible
, setInputFieldVisibleReq
, message
, messageReq
, panelAddButton
, panelAddButtonReq
, panelAddButtonStream
, panelAddButtonStreamReq
, panelAddInputField
, panelAddInputFieldReq
, panelAddInputFieldStream
, panelAddInputFieldStreamReq
, panelAddPanel
, panelAddPanelReq
, panelAddPanelStream
, panelAddPanelStreamReq
, panelAddText
, panelAddTextReq
, panelAddTextStream
, panelAddTextStreamReq
, panelRemove
, panelRemoveReq
, getPanelRectTransform
, getPanelRectTransformReq
, getPanelRectTransformStream
, getPanelRectTransformStreamReq
, getPanelVisible
, getPanelVisibleReq
, getPanelVisibleStream
, getPanelVisibleStreamReq
, setPanelVisible
, setPanelVisibleReq
, getRectTransformAnchorMax
, getRectTransformAnchorMaxReq
, getRectTransformAnchorMaxStream
, getRectTransformAnchorMaxStreamReq
, getRectTransformAnchorMin
, getRectTransformAnchorMinReq
, getRectTransformAnchorMinStream
, getRectTransformAnchorMinStreamReq
, getRectTransformLocalPosition
, getRectTransformLocalPositionReq
, getRectTransformLocalPositionStream
, getRectTransformLocalPositionStreamReq
, getRectTransformLowerLeft
, getRectTransformLowerLeftReq
, getRectTransformLowerLeftStream
, getRectTransformLowerLeftStreamReq
, getRectTransformPivot
, getRectTransformPivotReq
, getRectTransformPivotStream
, getRectTransformPivotStreamReq
, getRectTransformPosition
, getRectTransformPositionReq
, getRectTransformPositionStream
, getRectTransformPositionStreamReq
, getRectTransformRotation
, getRectTransformRotationReq
, getRectTransformRotationStream
, getRectTransformRotationStreamReq
, getRectTransformScale
, getRectTransformScaleReq
, getRectTransformScaleStream
, getRectTransformScaleStreamReq
, getRectTransformSize
, getRectTransformSizeReq
, getRectTransformSizeStream
, getRectTransformSizeStreamReq
, getRectTransformUpperRight
, getRectTransformUpperRightReq
, getRectTransformUpperRightStream
, getRectTransformUpperRightStreamReq
, setRectTransformAnchor
, setRectTransformAnchorReq
, setRectTransformAnchorMax
, setRectTransformAnchorMaxReq
, setRectTransformAnchorMin
, setRectTransformAnchorMinReq
, setRectTransformLocalPosition
, setRectTransformLocalPositionReq
, setRectTransformLowerLeft
, setRectTransformLowerLeftReq
, setRectTransformPivot
, setRectTransformPivotReq
, setRectTransformPosition
, setRectTransformPositionReq
, setRectTransformRotation
, setRectTransformRotationReq
, setRectTransformScale
, setRectTransformScaleReq
, setRectTransformSize
, setRectTransformSizeReq
, setRectTransformUpperRight
, setRectTransformUpperRightReq
, textRemove
, textRemoveReq
, getTextAlignment
, getTextAlignmentReq
, getTextAlignmentStream
, getTextAlignmentStreamReq
, getTextAvailableFonts
, getTextAvailableFontsReq
, getTextAvailableFontsStream
, getTextAvailableFontsStreamReq
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
, getTextRectTransform
, getTextRectTransformReq
, getTextRectTransformStream
, getTextRectTransformStreamReq
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
, setTextColor
, setTextColorReq
, setTextContent
, setTextContentReq
, setTextFont
, setTextFontReq
, setTextLineSpacing
, setTextLineSpacingReq
, setTextSize
, setTextSizeReq
, setTextStyle
, setTextStyleReq
, setTextVisible
, setTextVisibleReq
, getStockCanvas
, getStockCanvasReq
, getStockCanvasStream
, getStockCanvasStreamReq
) where

import qualified Data.Int
import qualified Data.Text

import KRPCHS.Internal.Requests
import KRPCHS.Internal.Requests.Call
import KRPCHS.Internal.Requests.Stream
import KRPCHS.Internal.SerializeUtils


{-|
A text label. See <see cref="M:UI.Panel.AddButton" />.
 -}
newtype Button = Button { buttonId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Button where
    encodePb = encodePb . buttonId

instance PbDecodable Button where
    decodePb b = Button <$> decodePb b

instance KRPCResponseExtractable Button

{-|
A canvas for user interface elements. See <see cref="M:UI.StockCanvas" /> and <see cref="M:UI.AddCanvas" />.
 -}
newtype Canvas = Canvas { canvasId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Canvas where
    encodePb = encodePb . canvasId

instance PbDecodable Canvas where
    decodePb b = Canvas <$> decodePb b

instance KRPCResponseExtractable Canvas

{-|
An input field. See <see cref="M:UI.Panel.AddInputField" />.
 -}
newtype InputField = InputField { inputFieldId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable InputField where
    encodePb = encodePb . inputFieldId

instance PbDecodable InputField where
    decodePb b = InputField <$> decodePb b

instance KRPCResponseExtractable InputField

{-|
A container for user interface elements. See <see cref="M:UI.Canvas.AddPanel" />.
 -}
newtype Panel = Panel { panelId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Panel where
    encodePb = encodePb . panelId

instance PbDecodable Panel where
    decodePb b = Panel <$> decodePb b

instance KRPCResponseExtractable Panel

{-|
A Unity engine Rect Transform for a UI object.
See the <a href="http://docs.unity3d.com/Manual/class-RectTransform.html">Unity manualfor more details.
 -}
newtype RectTransform = RectTransform { rectTransformId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable RectTransform where
    encodePb = encodePb . rectTransformId

instance PbDecodable RectTransform where
    decodePb b = RectTransform <$> decodePb b

instance KRPCResponseExtractable RectTransform

{-|
A text label. See <see cref="M:UI.Panel.AddText" />.
 -}
newtype Text = Text { textId :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable Text where
    encodePb = encodePb . textId

instance PbDecodable Text where
    decodePb b = Text <$> decodePb b

instance KRPCResponseExtractable Text


{-|
Font style.
 -}
data FontStyle
    = FontStyle'Normal
    | FontStyle'Bold
    | FontStyle'Italic
    | FontStyle'BoldAndItalic
    deriving (Show, Eq, Ord, Enum)

instance PbEncodable FontStyle where
    encodePb = encodePb . fromEnum

instance PbDecodable FontStyle where
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable FontStyle

{-|
Message position.
 -}
data MessagePosition
    = MessagePosition'BottomCenter
    | MessagePosition'TopCenter
    | MessagePosition'TopLeft
    | MessagePosition'TopRight
    deriving (Show, Eq, Ord, Enum)

instance PbEncodable MessagePosition where
    encodePb = encodePb . fromEnum

instance PbDecodable MessagePosition where
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable MessagePosition

{-|
Text alignment.
 -}
data TextAlignment
    = TextAlignment'Left
    | TextAlignment'Right
    | TextAlignment'Center
    deriving (Show, Eq, Ord, Enum)

instance PbEncodable TextAlignment where
    encodePb = encodePb . fromEnum

instance PbDecodable TextAlignment where
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable TextAlignment

{-|
Text alignment.
 -}
data TextAnchor
    = TextAnchor'LowerCenter
    | TextAnchor'LowerLeft
    | TextAnchor'LowerRight
    | TextAnchor'MiddleCenter
    | TextAnchor'MiddleLeft
    | TextAnchor'MiddleRight
    | TextAnchor'UpperCenter
    | TextAnchor'UpperLeft
    | TextAnchor'UpperRight
    deriving (Show, Eq, Ord, Enum)

instance PbEncodable TextAnchor where
    encodePb = encodePb . fromEnum

instance PbDecodable TextAnchor where
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable TextAnchor


{-|
Add a new canvas.If you want to add UI elements to KSPs stock UI canvas, use <see cref="M:UI.StockCanvas" />.
 -}
addCanvasReq :: KRPCCallReq (KRPCHS.UI.Canvas)
addCanvasReq  = makeCallReq "UI" "AddCanvas" []

addCanvas :: RPCContext (KRPCHS.UI.Canvas)
addCanvas  = simpleRequest $ addCanvasReq 

addCanvasStreamReq :: KRPCStreamReq (KRPCHS.UI.Canvas)
addCanvasStreamReq  = makeStreamReq $ addCanvasReq 

addCanvasStream :: RPCContext (KRPCStream (KRPCHS.UI.Canvas))
addCanvasStream  = requestAddStream $ addCanvasStreamReq  

{-|
Remove the UI object.
 -}
buttonRemoveReq :: KRPCHS.UI.Button -> KRPCCallReq ()
buttonRemoveReq thisArg = makeCallReq "UI" "Button_Remove" [makeArgument 0 thisArg]

buttonRemove :: KRPCHS.UI.Button -> RPCContext ()
buttonRemove thisArg = simpleRequest $ buttonRemoveReq thisArg 

{-|
Whether the button has been clicked.This property is set to true when the user clicks the button.
A client script should reset the property to false in order to detect subsequent button presses.
 -}
getButtonClickedReq :: KRPCHS.UI.Button -> KRPCCallReq (Bool)
getButtonClickedReq thisArg = makeCallReq "UI" "Button_get_Clicked" [makeArgument 0 thisArg]

getButtonClicked :: KRPCHS.UI.Button -> RPCContext (Bool)
getButtonClicked thisArg = simpleRequest $ getButtonClickedReq thisArg

getButtonClickedStreamReq :: KRPCHS.UI.Button -> KRPCStreamReq (Bool)
getButtonClickedStreamReq thisArg = makeStreamReq $ getButtonClickedReq thisArg

getButtonClickedStream :: KRPCHS.UI.Button -> RPCContext (KRPCStream (Bool))
getButtonClickedStream thisArg = requestAddStream $ getButtonClickedStreamReq thisArg 

{-|
The rect transform for the text.
 -}
getButtonRectTransformReq :: KRPCHS.UI.Button -> KRPCCallReq (KRPCHS.UI.RectTransform)
getButtonRectTransformReq thisArg = makeCallReq "UI" "Button_get_RectTransform" [makeArgument 0 thisArg]

getButtonRectTransform :: KRPCHS.UI.Button -> RPCContext (KRPCHS.UI.RectTransform)
getButtonRectTransform thisArg = simpleRequest $ getButtonRectTransformReq thisArg

getButtonRectTransformStreamReq :: KRPCHS.UI.Button -> KRPCStreamReq (KRPCHS.UI.RectTransform)
getButtonRectTransformStreamReq thisArg = makeStreamReq $ getButtonRectTransformReq thisArg

getButtonRectTransformStream :: KRPCHS.UI.Button -> RPCContext (KRPCStream (KRPCHS.UI.RectTransform))
getButtonRectTransformStream thisArg = requestAddStream $ getButtonRectTransformStreamReq thisArg 

{-|
The text for the button.
 -}
getButtonTextReq :: KRPCHS.UI.Button -> KRPCCallReq (KRPCHS.UI.Text)
getButtonTextReq thisArg = makeCallReq "UI" "Button_get_Text" [makeArgument 0 thisArg]

getButtonText :: KRPCHS.UI.Button -> RPCContext (KRPCHS.UI.Text)
getButtonText thisArg = simpleRequest $ getButtonTextReq thisArg

getButtonTextStreamReq :: KRPCHS.UI.Button -> KRPCStreamReq (KRPCHS.UI.Text)
getButtonTextStreamReq thisArg = makeStreamReq $ getButtonTextReq thisArg

getButtonTextStream :: KRPCHS.UI.Button -> RPCContext (KRPCStream (KRPCHS.UI.Text))
getButtonTextStream thisArg = requestAddStream $ getButtonTextStreamReq thisArg 

{-|
Whether the UI object is visible.
 -}
getButtonVisibleReq :: KRPCHS.UI.Button -> KRPCCallReq (Bool)
getButtonVisibleReq thisArg = makeCallReq "UI" "Button_get_Visible" [makeArgument 0 thisArg]

getButtonVisible :: KRPCHS.UI.Button -> RPCContext (Bool)
getButtonVisible thisArg = simpleRequest $ getButtonVisibleReq thisArg

getButtonVisibleStreamReq :: KRPCHS.UI.Button -> KRPCStreamReq (Bool)
getButtonVisibleStreamReq thisArg = makeStreamReq $ getButtonVisibleReq thisArg

getButtonVisibleStream :: KRPCHS.UI.Button -> RPCContext (KRPCStream (Bool))
getButtonVisibleStream thisArg = requestAddStream $ getButtonVisibleStreamReq thisArg 

{-|
Whether the button has been clicked.This property is set to true when the user clicks the button.
A client script should reset the property to false in order to detect subsequent button presses.
 -}
setButtonClickedReq :: KRPCHS.UI.Button -> Bool -> KRPCCallReq ()
setButtonClickedReq thisArg valueArg = makeCallReq "UI" "Button_set_Clicked" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setButtonClicked :: KRPCHS.UI.Button -> Bool -> RPCContext ()
setButtonClicked thisArg valueArg = simpleRequest $ setButtonClickedReq thisArg valueArg 

{-|
Whether the UI object is visible.
 -}
setButtonVisibleReq :: KRPCHS.UI.Button -> Bool -> KRPCCallReq ()
setButtonVisibleReq thisArg valueArg = makeCallReq "UI" "Button_set_Visible" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setButtonVisible :: KRPCHS.UI.Button -> Bool -> RPCContext ()
setButtonVisible thisArg valueArg = simpleRequest $ setButtonVisibleReq thisArg valueArg 

{-|
Add a button to the canvas.<param name="content">The label for the button.<param name="visible">Whether the button is visible.
 -}
canvasAddButtonReq :: KRPCHS.UI.Canvas -> Data.Text.Text -> Bool -> KRPCCallReq (KRPCHS.UI.Button)
canvasAddButtonReq thisArg contentArg visibleArg = makeCallReq "UI" "Canvas_AddButton" [makeArgument 0 thisArg, makeArgument 1 contentArg, makeArgument 2 visibleArg]

canvasAddButton :: KRPCHS.UI.Canvas -> Data.Text.Text -> Bool -> RPCContext (KRPCHS.UI.Button)
canvasAddButton thisArg contentArg visibleArg = simpleRequest $ canvasAddButtonReq thisArg contentArg visibleArg

canvasAddButtonStreamReq :: KRPCHS.UI.Canvas -> Data.Text.Text -> Bool -> KRPCStreamReq (KRPCHS.UI.Button)
canvasAddButtonStreamReq thisArg contentArg visibleArg = makeStreamReq $ canvasAddButtonReq thisArg contentArg visibleArg

canvasAddButtonStream :: KRPCHS.UI.Canvas -> Data.Text.Text -> Bool -> RPCContext (KRPCStream (KRPCHS.UI.Button))
canvasAddButtonStream thisArg contentArg visibleArg = requestAddStream $ canvasAddButtonStreamReq thisArg contentArg visibleArg 

{-|
Add an input field to the canvas.<param name="visible">Whether the input field is visible.
 -}
canvasAddInputFieldReq :: KRPCHS.UI.Canvas -> Bool -> KRPCCallReq (KRPCHS.UI.InputField)
canvasAddInputFieldReq thisArg visibleArg = makeCallReq "UI" "Canvas_AddInputField" [makeArgument 0 thisArg, makeArgument 1 visibleArg]

canvasAddInputField :: KRPCHS.UI.Canvas -> Bool -> RPCContext (KRPCHS.UI.InputField)
canvasAddInputField thisArg visibleArg = simpleRequest $ canvasAddInputFieldReq thisArg visibleArg

canvasAddInputFieldStreamReq :: KRPCHS.UI.Canvas -> Bool -> KRPCStreamReq (KRPCHS.UI.InputField)
canvasAddInputFieldStreamReq thisArg visibleArg = makeStreamReq $ canvasAddInputFieldReq thisArg visibleArg

canvasAddInputFieldStream :: KRPCHS.UI.Canvas -> Bool -> RPCContext (KRPCStream (KRPCHS.UI.InputField))
canvasAddInputFieldStream thisArg visibleArg = requestAddStream $ canvasAddInputFieldStreamReq thisArg visibleArg 

{-|
Create a new container for user interface elements.<param name="visible">Whether the panel is visible.
 -}
canvasAddPanelReq :: KRPCHS.UI.Canvas -> Bool -> KRPCCallReq (KRPCHS.UI.Panel)
canvasAddPanelReq thisArg visibleArg = makeCallReq "UI" "Canvas_AddPanel" [makeArgument 0 thisArg, makeArgument 1 visibleArg]

canvasAddPanel :: KRPCHS.UI.Canvas -> Bool -> RPCContext (KRPCHS.UI.Panel)
canvasAddPanel thisArg visibleArg = simpleRequest $ canvasAddPanelReq thisArg visibleArg

canvasAddPanelStreamReq :: KRPCHS.UI.Canvas -> Bool -> KRPCStreamReq (KRPCHS.UI.Panel)
canvasAddPanelStreamReq thisArg visibleArg = makeStreamReq $ canvasAddPanelReq thisArg visibleArg

canvasAddPanelStream :: KRPCHS.UI.Canvas -> Bool -> RPCContext (KRPCStream (KRPCHS.UI.Panel))
canvasAddPanelStream thisArg visibleArg = requestAddStream $ canvasAddPanelStreamReq thisArg visibleArg 

{-|
Add text to the canvas.<param name="content">The text.<param name="visible">Whether the text is visible.
 -}
canvasAddTextReq :: KRPCHS.UI.Canvas -> Data.Text.Text -> Bool -> KRPCCallReq (KRPCHS.UI.Text)
canvasAddTextReq thisArg contentArg visibleArg = makeCallReq "UI" "Canvas_AddText" [makeArgument 0 thisArg, makeArgument 1 contentArg, makeArgument 2 visibleArg]

canvasAddText :: KRPCHS.UI.Canvas -> Data.Text.Text -> Bool -> RPCContext (KRPCHS.UI.Text)
canvasAddText thisArg contentArg visibleArg = simpleRequest $ canvasAddTextReq thisArg contentArg visibleArg

canvasAddTextStreamReq :: KRPCHS.UI.Canvas -> Data.Text.Text -> Bool -> KRPCStreamReq (KRPCHS.UI.Text)
canvasAddTextStreamReq thisArg contentArg visibleArg = makeStreamReq $ canvasAddTextReq thisArg contentArg visibleArg

canvasAddTextStream :: KRPCHS.UI.Canvas -> Data.Text.Text -> Bool -> RPCContext (KRPCStream (KRPCHS.UI.Text))
canvasAddTextStream thisArg contentArg visibleArg = requestAddStream $ canvasAddTextStreamReq thisArg contentArg visibleArg 

{-|
Remove the UI object.
 -}
canvasRemoveReq :: KRPCHS.UI.Canvas -> KRPCCallReq ()
canvasRemoveReq thisArg = makeCallReq "UI" "Canvas_Remove" [makeArgument 0 thisArg]

canvasRemove :: KRPCHS.UI.Canvas -> RPCContext ()
canvasRemove thisArg = simpleRequest $ canvasRemoveReq thisArg 

{-|
The rect transform for the canvas.
 -}
getCanvasRectTransformReq :: KRPCHS.UI.Canvas -> KRPCCallReq (KRPCHS.UI.RectTransform)
getCanvasRectTransformReq thisArg = makeCallReq "UI" "Canvas_get_RectTransform" [makeArgument 0 thisArg]

getCanvasRectTransform :: KRPCHS.UI.Canvas -> RPCContext (KRPCHS.UI.RectTransform)
getCanvasRectTransform thisArg = simpleRequest $ getCanvasRectTransformReq thisArg

getCanvasRectTransformStreamReq :: KRPCHS.UI.Canvas -> KRPCStreamReq (KRPCHS.UI.RectTransform)
getCanvasRectTransformStreamReq thisArg = makeStreamReq $ getCanvasRectTransformReq thisArg

getCanvasRectTransformStream :: KRPCHS.UI.Canvas -> RPCContext (KRPCStream (KRPCHS.UI.RectTransform))
getCanvasRectTransformStream thisArg = requestAddStream $ getCanvasRectTransformStreamReq thisArg 

{-|
Whether the UI object is visible.
 -}
getCanvasVisibleReq :: KRPCHS.UI.Canvas -> KRPCCallReq (Bool)
getCanvasVisibleReq thisArg = makeCallReq "UI" "Canvas_get_Visible" [makeArgument 0 thisArg]

getCanvasVisible :: KRPCHS.UI.Canvas -> RPCContext (Bool)
getCanvasVisible thisArg = simpleRequest $ getCanvasVisibleReq thisArg

getCanvasVisibleStreamReq :: KRPCHS.UI.Canvas -> KRPCStreamReq (Bool)
getCanvasVisibleStreamReq thisArg = makeStreamReq $ getCanvasVisibleReq thisArg

getCanvasVisibleStream :: KRPCHS.UI.Canvas -> RPCContext (KRPCStream (Bool))
getCanvasVisibleStream thisArg = requestAddStream $ getCanvasVisibleStreamReq thisArg 

{-|
Whether the UI object is visible.
 -}
setCanvasVisibleReq :: KRPCHS.UI.Canvas -> Bool -> KRPCCallReq ()
setCanvasVisibleReq thisArg valueArg = makeCallReq "UI" "Canvas_set_Visible" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setCanvasVisible :: KRPCHS.UI.Canvas -> Bool -> RPCContext ()
setCanvasVisible thisArg valueArg = simpleRequest $ setCanvasVisibleReq thisArg valueArg 

{-|
Remove all user interface elements.<param name="clientOnly">If true, only remove objects created by the calling client.
 -}
clearReq :: Bool -> KRPCCallReq ()
clearReq clientOnlyArg = makeCallReq "UI" "Clear" [makeArgument 0 clientOnlyArg]

clear :: Bool -> RPCContext ()
clear clientOnlyArg = simpleRequest $ clearReq clientOnlyArg 

{-|
Remove the UI object.
 -}
inputFieldRemoveReq :: KRPCHS.UI.InputField -> KRPCCallReq ()
inputFieldRemoveReq thisArg = makeCallReq "UI" "InputField_Remove" [makeArgument 0 thisArg]

inputFieldRemove :: KRPCHS.UI.InputField -> RPCContext ()
inputFieldRemove thisArg = simpleRequest $ inputFieldRemoveReq thisArg 

{-|
Whether the input field has been changed.This property is set to true when the user modifies the value of the input field.
A client script should reset the property to false in order to detect subsequent changes.
 -}
getInputFieldChangedReq :: KRPCHS.UI.InputField -> KRPCCallReq (Bool)
getInputFieldChangedReq thisArg = makeCallReq "UI" "InputField_get_Changed" [makeArgument 0 thisArg]

getInputFieldChanged :: KRPCHS.UI.InputField -> RPCContext (Bool)
getInputFieldChanged thisArg = simpleRequest $ getInputFieldChangedReq thisArg

getInputFieldChangedStreamReq :: KRPCHS.UI.InputField -> KRPCStreamReq (Bool)
getInputFieldChangedStreamReq thisArg = makeStreamReq $ getInputFieldChangedReq thisArg

getInputFieldChangedStream :: KRPCHS.UI.InputField -> RPCContext (KRPCStream (Bool))
getInputFieldChangedStream thisArg = requestAddStream $ getInputFieldChangedStreamReq thisArg 

{-|
The rect transform for the input field.
 -}
getInputFieldRectTransformReq :: KRPCHS.UI.InputField -> KRPCCallReq (KRPCHS.UI.RectTransform)
getInputFieldRectTransformReq thisArg = makeCallReq "UI" "InputField_get_RectTransform" [makeArgument 0 thisArg]

getInputFieldRectTransform :: KRPCHS.UI.InputField -> RPCContext (KRPCHS.UI.RectTransform)
getInputFieldRectTransform thisArg = simpleRequest $ getInputFieldRectTransformReq thisArg

getInputFieldRectTransformStreamReq :: KRPCHS.UI.InputField -> KRPCStreamReq (KRPCHS.UI.RectTransform)
getInputFieldRectTransformStreamReq thisArg = makeStreamReq $ getInputFieldRectTransformReq thisArg

getInputFieldRectTransformStream :: KRPCHS.UI.InputField -> RPCContext (KRPCStream (KRPCHS.UI.RectTransform))
getInputFieldRectTransformStream thisArg = requestAddStream $ getInputFieldRectTransformStreamReq thisArg 

{-|
The text component of the input field.Use <see cref="M:UI.InputField.Value" /> to get and set the value in the field.
This object can be used to alter the style of the input field's text.
 -}
getInputFieldTextReq :: KRPCHS.UI.InputField -> KRPCCallReq (KRPCHS.UI.Text)
getInputFieldTextReq thisArg = makeCallReq "UI" "InputField_get_Text" [makeArgument 0 thisArg]

getInputFieldText :: KRPCHS.UI.InputField -> RPCContext (KRPCHS.UI.Text)
getInputFieldText thisArg = simpleRequest $ getInputFieldTextReq thisArg

getInputFieldTextStreamReq :: KRPCHS.UI.InputField -> KRPCStreamReq (KRPCHS.UI.Text)
getInputFieldTextStreamReq thisArg = makeStreamReq $ getInputFieldTextReq thisArg

getInputFieldTextStream :: KRPCHS.UI.InputField -> RPCContext (KRPCStream (KRPCHS.UI.Text))
getInputFieldTextStream thisArg = requestAddStream $ getInputFieldTextStreamReq thisArg 

{-|
The value of the input field.
 -}
getInputFieldValueReq :: KRPCHS.UI.InputField -> KRPCCallReq (Data.Text.Text)
getInputFieldValueReq thisArg = makeCallReq "UI" "InputField_get_Value" [makeArgument 0 thisArg]

getInputFieldValue :: KRPCHS.UI.InputField -> RPCContext (Data.Text.Text)
getInputFieldValue thisArg = simpleRequest $ getInputFieldValueReq thisArg

getInputFieldValueStreamReq :: KRPCHS.UI.InputField -> KRPCStreamReq (Data.Text.Text)
getInputFieldValueStreamReq thisArg = makeStreamReq $ getInputFieldValueReq thisArg

getInputFieldValueStream :: KRPCHS.UI.InputField -> RPCContext (KRPCStream (Data.Text.Text))
getInputFieldValueStream thisArg = requestAddStream $ getInputFieldValueStreamReq thisArg 

{-|
Whether the UI object is visible.
 -}
getInputFieldVisibleReq :: KRPCHS.UI.InputField -> KRPCCallReq (Bool)
getInputFieldVisibleReq thisArg = makeCallReq "UI" "InputField_get_Visible" [makeArgument 0 thisArg]

getInputFieldVisible :: KRPCHS.UI.InputField -> RPCContext (Bool)
getInputFieldVisible thisArg = simpleRequest $ getInputFieldVisibleReq thisArg

getInputFieldVisibleStreamReq :: KRPCHS.UI.InputField -> KRPCStreamReq (Bool)
getInputFieldVisibleStreamReq thisArg = makeStreamReq $ getInputFieldVisibleReq thisArg

getInputFieldVisibleStream :: KRPCHS.UI.InputField -> RPCContext (KRPCStream (Bool))
getInputFieldVisibleStream thisArg = requestAddStream $ getInputFieldVisibleStreamReq thisArg 

{-|
Whether the input field has been changed.This property is set to true when the user modifies the value of the input field.
A client script should reset the property to false in order to detect subsequent changes.
 -}
setInputFieldChangedReq :: KRPCHS.UI.InputField -> Bool -> KRPCCallReq ()
setInputFieldChangedReq thisArg valueArg = makeCallReq "UI" "InputField_set_Changed" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setInputFieldChanged :: KRPCHS.UI.InputField -> Bool -> RPCContext ()
setInputFieldChanged thisArg valueArg = simpleRequest $ setInputFieldChangedReq thisArg valueArg 

{-|
The value of the input field.
 -}
setInputFieldValueReq :: KRPCHS.UI.InputField -> Data.Text.Text -> KRPCCallReq ()
setInputFieldValueReq thisArg valueArg = makeCallReq "UI" "InputField_set_Value" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setInputFieldValue :: KRPCHS.UI.InputField -> Data.Text.Text -> RPCContext ()
setInputFieldValue thisArg valueArg = simpleRequest $ setInputFieldValueReq thisArg valueArg 

{-|
Whether the UI object is visible.
 -}
setInputFieldVisibleReq :: KRPCHS.UI.InputField -> Bool -> KRPCCallReq ()
setInputFieldVisibleReq thisArg valueArg = makeCallReq "UI" "InputField_set_Visible" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setInputFieldVisible :: KRPCHS.UI.InputField -> Bool -> RPCContext ()
setInputFieldVisible thisArg valueArg = simpleRequest $ setInputFieldVisibleReq thisArg valueArg 

{-|
Display a message on the screen.The message appears just like a stock message, for example quicksave or quickload messages.<param name="content">Message content.<param name="duration">Duration before the message disappears, in seconds.<param name="position">Position to display the message.
 -}
messageReq :: Data.Text.Text -> Float -> KRPCHS.UI.MessagePosition -> KRPCCallReq ()
messageReq contentArg durationArg positionArg = makeCallReq "UI" "Message" [makeArgument 0 contentArg, makeArgument 1 durationArg, makeArgument 2 positionArg]

message :: Data.Text.Text -> Float -> KRPCHS.UI.MessagePosition -> RPCContext ()
message contentArg durationArg positionArg = simpleRequest $ messageReq contentArg durationArg positionArg 

{-|
Add a button to the panel.<param name="content">The label for the button.<param name="visible">Whether the button is visible.
 -}
panelAddButtonReq :: KRPCHS.UI.Panel -> Data.Text.Text -> Bool -> KRPCCallReq (KRPCHS.UI.Button)
panelAddButtonReq thisArg contentArg visibleArg = makeCallReq "UI" "Panel_AddButton" [makeArgument 0 thisArg, makeArgument 1 contentArg, makeArgument 2 visibleArg]

panelAddButton :: KRPCHS.UI.Panel -> Data.Text.Text -> Bool -> RPCContext (KRPCHS.UI.Button)
panelAddButton thisArg contentArg visibleArg = simpleRequest $ panelAddButtonReq thisArg contentArg visibleArg

panelAddButtonStreamReq :: KRPCHS.UI.Panel -> Data.Text.Text -> Bool -> KRPCStreamReq (KRPCHS.UI.Button)
panelAddButtonStreamReq thisArg contentArg visibleArg = makeStreamReq $ panelAddButtonReq thisArg contentArg visibleArg

panelAddButtonStream :: KRPCHS.UI.Panel -> Data.Text.Text -> Bool -> RPCContext (KRPCStream (KRPCHS.UI.Button))
panelAddButtonStream thisArg contentArg visibleArg = requestAddStream $ panelAddButtonStreamReq thisArg contentArg visibleArg 

{-|
Add an input field to the panel.<param name="visible">Whether the input field is visible.
 -}
panelAddInputFieldReq :: KRPCHS.UI.Panel -> Bool -> KRPCCallReq (KRPCHS.UI.InputField)
panelAddInputFieldReq thisArg visibleArg = makeCallReq "UI" "Panel_AddInputField" [makeArgument 0 thisArg, makeArgument 1 visibleArg]

panelAddInputField :: KRPCHS.UI.Panel -> Bool -> RPCContext (KRPCHS.UI.InputField)
panelAddInputField thisArg visibleArg = simpleRequest $ panelAddInputFieldReq thisArg visibleArg

panelAddInputFieldStreamReq :: KRPCHS.UI.Panel -> Bool -> KRPCStreamReq (KRPCHS.UI.InputField)
panelAddInputFieldStreamReq thisArg visibleArg = makeStreamReq $ panelAddInputFieldReq thisArg visibleArg

panelAddInputFieldStream :: KRPCHS.UI.Panel -> Bool -> RPCContext (KRPCStream (KRPCHS.UI.InputField))
panelAddInputFieldStream thisArg visibleArg = requestAddStream $ panelAddInputFieldStreamReq thisArg visibleArg 

{-|
Create a panel within this panel.<param name="visible">Whether the new panel is visible.
 -}
panelAddPanelReq :: KRPCHS.UI.Panel -> Bool -> KRPCCallReq (KRPCHS.UI.Panel)
panelAddPanelReq thisArg visibleArg = makeCallReq "UI" "Panel_AddPanel" [makeArgument 0 thisArg, makeArgument 1 visibleArg]

panelAddPanel :: KRPCHS.UI.Panel -> Bool -> RPCContext (KRPCHS.UI.Panel)
panelAddPanel thisArg visibleArg = simpleRequest $ panelAddPanelReq thisArg visibleArg

panelAddPanelStreamReq :: KRPCHS.UI.Panel -> Bool -> KRPCStreamReq (KRPCHS.UI.Panel)
panelAddPanelStreamReq thisArg visibleArg = makeStreamReq $ panelAddPanelReq thisArg visibleArg

panelAddPanelStream :: KRPCHS.UI.Panel -> Bool -> RPCContext (KRPCStream (KRPCHS.UI.Panel))
panelAddPanelStream thisArg visibleArg = requestAddStream $ panelAddPanelStreamReq thisArg visibleArg 

{-|
Add text to the panel.<param name="content">The text.<param name="visible">Whether the text is visible.
 -}
panelAddTextReq :: KRPCHS.UI.Panel -> Data.Text.Text -> Bool -> KRPCCallReq (KRPCHS.UI.Text)
panelAddTextReq thisArg contentArg visibleArg = makeCallReq "UI" "Panel_AddText" [makeArgument 0 thisArg, makeArgument 1 contentArg, makeArgument 2 visibleArg]

panelAddText :: KRPCHS.UI.Panel -> Data.Text.Text -> Bool -> RPCContext (KRPCHS.UI.Text)
panelAddText thisArg contentArg visibleArg = simpleRequest $ panelAddTextReq thisArg contentArg visibleArg

panelAddTextStreamReq :: KRPCHS.UI.Panel -> Data.Text.Text -> Bool -> KRPCStreamReq (KRPCHS.UI.Text)
panelAddTextStreamReq thisArg contentArg visibleArg = makeStreamReq $ panelAddTextReq thisArg contentArg visibleArg

panelAddTextStream :: KRPCHS.UI.Panel -> Data.Text.Text -> Bool -> RPCContext (KRPCStream (KRPCHS.UI.Text))
panelAddTextStream thisArg contentArg visibleArg = requestAddStream $ panelAddTextStreamReq thisArg contentArg visibleArg 

{-|
Remove the UI object.
 -}
panelRemoveReq :: KRPCHS.UI.Panel -> KRPCCallReq ()
panelRemoveReq thisArg = makeCallReq "UI" "Panel_Remove" [makeArgument 0 thisArg]

panelRemove :: KRPCHS.UI.Panel -> RPCContext ()
panelRemove thisArg = simpleRequest $ panelRemoveReq thisArg 

{-|
The rect transform for the panel.
 -}
getPanelRectTransformReq :: KRPCHS.UI.Panel -> KRPCCallReq (KRPCHS.UI.RectTransform)
getPanelRectTransformReq thisArg = makeCallReq "UI" "Panel_get_RectTransform" [makeArgument 0 thisArg]

getPanelRectTransform :: KRPCHS.UI.Panel -> RPCContext (KRPCHS.UI.RectTransform)
getPanelRectTransform thisArg = simpleRequest $ getPanelRectTransformReq thisArg

getPanelRectTransformStreamReq :: KRPCHS.UI.Panel -> KRPCStreamReq (KRPCHS.UI.RectTransform)
getPanelRectTransformStreamReq thisArg = makeStreamReq $ getPanelRectTransformReq thisArg

getPanelRectTransformStream :: KRPCHS.UI.Panel -> RPCContext (KRPCStream (KRPCHS.UI.RectTransform))
getPanelRectTransformStream thisArg = requestAddStream $ getPanelRectTransformStreamReq thisArg 

{-|
Whether the UI object is visible.
 -}
getPanelVisibleReq :: KRPCHS.UI.Panel -> KRPCCallReq (Bool)
getPanelVisibleReq thisArg = makeCallReq "UI" "Panel_get_Visible" [makeArgument 0 thisArg]

getPanelVisible :: KRPCHS.UI.Panel -> RPCContext (Bool)
getPanelVisible thisArg = simpleRequest $ getPanelVisibleReq thisArg

getPanelVisibleStreamReq :: KRPCHS.UI.Panel -> KRPCStreamReq (Bool)
getPanelVisibleStreamReq thisArg = makeStreamReq $ getPanelVisibleReq thisArg

getPanelVisibleStream :: KRPCHS.UI.Panel -> RPCContext (KRPCStream (Bool))
getPanelVisibleStream thisArg = requestAddStream $ getPanelVisibleStreamReq thisArg 

{-|
Whether the UI object is visible.
 -}
setPanelVisibleReq :: KRPCHS.UI.Panel -> Bool -> KRPCCallReq ()
setPanelVisibleReq thisArg valueArg = makeCallReq "UI" "Panel_set_Visible" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setPanelVisible :: KRPCHS.UI.Panel -> Bool -> RPCContext ()
setPanelVisible thisArg valueArg = simpleRequest $ setPanelVisibleReq thisArg valueArg 

{-|
The anchor point for the lower left corner of the rectangle defined as a fraction of the size of the parent rectangle.
 -}
getRectTransformAnchorMaxReq :: KRPCHS.UI.RectTransform -> KRPCCallReq ((Double, Double))
getRectTransformAnchorMaxReq thisArg = makeCallReq "UI" "RectTransform_get_AnchorMax" [makeArgument 0 thisArg]

getRectTransformAnchorMax :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double))
getRectTransformAnchorMax thisArg = simpleRequest $ getRectTransformAnchorMaxReq thisArg

getRectTransformAnchorMaxStreamReq :: KRPCHS.UI.RectTransform -> KRPCStreamReq ((Double, Double))
getRectTransformAnchorMaxStreamReq thisArg = makeStreamReq $ getRectTransformAnchorMaxReq thisArg

getRectTransformAnchorMaxStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double)))
getRectTransformAnchorMaxStream thisArg = requestAddStream $ getRectTransformAnchorMaxStreamReq thisArg 

{-|
The anchor point for the upper right corner of the rectangle defined as a fraction of the size of the parent rectangle.
 -}
getRectTransformAnchorMinReq :: KRPCHS.UI.RectTransform -> KRPCCallReq ((Double, Double))
getRectTransformAnchorMinReq thisArg = makeCallReq "UI" "RectTransform_get_AnchorMin" [makeArgument 0 thisArg]

getRectTransformAnchorMin :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double))
getRectTransformAnchorMin thisArg = simpleRequest $ getRectTransformAnchorMinReq thisArg

getRectTransformAnchorMinStreamReq :: KRPCHS.UI.RectTransform -> KRPCStreamReq ((Double, Double))
getRectTransformAnchorMinStreamReq thisArg = makeStreamReq $ getRectTransformAnchorMinReq thisArg

getRectTransformAnchorMinStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double)))
getRectTransformAnchorMinStream thisArg = requestAddStream $ getRectTransformAnchorMinStreamReq thisArg 

{-|
Position of the rectangles pivot point relative to the anchors.
 -}
getRectTransformLocalPositionReq :: KRPCHS.UI.RectTransform -> KRPCCallReq ((Double, Double, Double))
getRectTransformLocalPositionReq thisArg = makeCallReq "UI" "RectTransform_get_LocalPosition" [makeArgument 0 thisArg]

getRectTransformLocalPosition :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double, Double))
getRectTransformLocalPosition thisArg = simpleRequest $ getRectTransformLocalPositionReq thisArg

getRectTransformLocalPositionStreamReq :: KRPCHS.UI.RectTransform -> KRPCStreamReq ((Double, Double, Double))
getRectTransformLocalPositionStreamReq thisArg = makeStreamReq $ getRectTransformLocalPositionReq thisArg

getRectTransformLocalPositionStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double, Double)))
getRectTransformLocalPositionStream thisArg = requestAddStream $ getRectTransformLocalPositionStreamReq thisArg 

{-|
Position of the rectangles lower left corner relative to the anchors.
 -}
getRectTransformLowerLeftReq :: KRPCHS.UI.RectTransform -> KRPCCallReq ((Double, Double))
getRectTransformLowerLeftReq thisArg = makeCallReq "UI" "RectTransform_get_LowerLeft" [makeArgument 0 thisArg]

getRectTransformLowerLeft :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double))
getRectTransformLowerLeft thisArg = simpleRequest $ getRectTransformLowerLeftReq thisArg

getRectTransformLowerLeftStreamReq :: KRPCHS.UI.RectTransform -> KRPCStreamReq ((Double, Double))
getRectTransformLowerLeftStreamReq thisArg = makeStreamReq $ getRectTransformLowerLeftReq thisArg

getRectTransformLowerLeftStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double)))
getRectTransformLowerLeftStream thisArg = requestAddStream $ getRectTransformLowerLeftStreamReq thisArg 

{-|
Location of the pivot point around which the rectangle rotates, defined as a fraction of the size of the rectangle itself.
 -}
getRectTransformPivotReq :: KRPCHS.UI.RectTransform -> KRPCCallReq ((Double, Double))
getRectTransformPivotReq thisArg = makeCallReq "UI" "RectTransform_get_Pivot" [makeArgument 0 thisArg]

getRectTransformPivot :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double))
getRectTransformPivot thisArg = simpleRequest $ getRectTransformPivotReq thisArg

getRectTransformPivotStreamReq :: KRPCHS.UI.RectTransform -> KRPCStreamReq ((Double, Double))
getRectTransformPivotStreamReq thisArg = makeStreamReq $ getRectTransformPivotReq thisArg

getRectTransformPivotStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double)))
getRectTransformPivotStream thisArg = requestAddStream $ getRectTransformPivotStreamReq thisArg 

{-|
Position of the rectangles pivot point relative to the anchors.
 -}
getRectTransformPositionReq :: KRPCHS.UI.RectTransform -> KRPCCallReq ((Double, Double))
getRectTransformPositionReq thisArg = makeCallReq "UI" "RectTransform_get_Position" [makeArgument 0 thisArg]

getRectTransformPosition :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double))
getRectTransformPosition thisArg = simpleRequest $ getRectTransformPositionReq thisArg

getRectTransformPositionStreamReq :: KRPCHS.UI.RectTransform -> KRPCStreamReq ((Double, Double))
getRectTransformPositionStreamReq thisArg = makeStreamReq $ getRectTransformPositionReq thisArg

getRectTransformPositionStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double)))
getRectTransformPositionStream thisArg = requestAddStream $ getRectTransformPositionStreamReq thisArg 

{-|
Rotation, as a quaternion, of the object around its pivot point.
 -}
getRectTransformRotationReq :: KRPCHS.UI.RectTransform -> KRPCCallReq ((Double, Double, Double, Double))
getRectTransformRotationReq thisArg = makeCallReq "UI" "RectTransform_get_Rotation" [makeArgument 0 thisArg]

getRectTransformRotation :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double, Double, Double))
getRectTransformRotation thisArg = simpleRequest $ getRectTransformRotationReq thisArg

getRectTransformRotationStreamReq :: KRPCHS.UI.RectTransform -> KRPCStreamReq ((Double, Double, Double, Double))
getRectTransformRotationStreamReq thisArg = makeStreamReq $ getRectTransformRotationReq thisArg

getRectTransformRotationStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
getRectTransformRotationStream thisArg = requestAddStream $ getRectTransformRotationStreamReq thisArg 

{-|
Scale factor applied to the object in the x, y and z dimensions.
 -}
getRectTransformScaleReq :: KRPCHS.UI.RectTransform -> KRPCCallReq ((Double, Double, Double))
getRectTransformScaleReq thisArg = makeCallReq "UI" "RectTransform_get_Scale" [makeArgument 0 thisArg]

getRectTransformScale :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double, Double))
getRectTransformScale thisArg = simpleRequest $ getRectTransformScaleReq thisArg

getRectTransformScaleStreamReq :: KRPCHS.UI.RectTransform -> KRPCStreamReq ((Double, Double, Double))
getRectTransformScaleStreamReq thisArg = makeStreamReq $ getRectTransformScaleReq thisArg

getRectTransformScaleStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double, Double)))
getRectTransformScaleStream thisArg = requestAddStream $ getRectTransformScaleStreamReq thisArg 

{-|
Width and height of the rectangle.
 -}
getRectTransformSizeReq :: KRPCHS.UI.RectTransform -> KRPCCallReq ((Double, Double))
getRectTransformSizeReq thisArg = makeCallReq "UI" "RectTransform_get_Size" [makeArgument 0 thisArg]

getRectTransformSize :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double))
getRectTransformSize thisArg = simpleRequest $ getRectTransformSizeReq thisArg

getRectTransformSizeStreamReq :: KRPCHS.UI.RectTransform -> KRPCStreamReq ((Double, Double))
getRectTransformSizeStreamReq thisArg = makeStreamReq $ getRectTransformSizeReq thisArg

getRectTransformSizeStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double)))
getRectTransformSizeStream thisArg = requestAddStream $ getRectTransformSizeStreamReq thisArg 

{-|
Position of the rectangles upper right corner relative to the anchors.
 -}
getRectTransformUpperRightReq :: KRPCHS.UI.RectTransform -> KRPCCallReq ((Double, Double))
getRectTransformUpperRightReq thisArg = makeCallReq "UI" "RectTransform_get_UpperRight" [makeArgument 0 thisArg]

getRectTransformUpperRight :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double))
getRectTransformUpperRight thisArg = simpleRequest $ getRectTransformUpperRightReq thisArg

getRectTransformUpperRightStreamReq :: KRPCHS.UI.RectTransform -> KRPCStreamReq ((Double, Double))
getRectTransformUpperRightStreamReq thisArg = makeStreamReq $ getRectTransformUpperRightReq thisArg

getRectTransformUpperRightStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double)))
getRectTransformUpperRightStream thisArg = requestAddStream $ getRectTransformUpperRightStreamReq thisArg 

{-|
Set the minimum and maximum anchor points as a fraction of the size of the parent rectangle.
 -}
setRectTransformAnchorReq :: KRPCHS.UI.RectTransform -> (Double, Double) -> KRPCCallReq ()
setRectTransformAnchorReq thisArg valueArg = makeCallReq "UI" "RectTransform_set_Anchor" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setRectTransformAnchor :: KRPCHS.UI.RectTransform -> (Double, Double) -> RPCContext ()
setRectTransformAnchor thisArg valueArg = simpleRequest $ setRectTransformAnchorReq thisArg valueArg 

{-|
The anchor point for the lower left corner of the rectangle defined as a fraction of the size of the parent rectangle.
 -}
setRectTransformAnchorMaxReq :: KRPCHS.UI.RectTransform -> (Double, Double) -> KRPCCallReq ()
setRectTransformAnchorMaxReq thisArg valueArg = makeCallReq "UI" "RectTransform_set_AnchorMax" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setRectTransformAnchorMax :: KRPCHS.UI.RectTransform -> (Double, Double) -> RPCContext ()
setRectTransformAnchorMax thisArg valueArg = simpleRequest $ setRectTransformAnchorMaxReq thisArg valueArg 

{-|
The anchor point for the upper right corner of the rectangle defined as a fraction of the size of the parent rectangle.
 -}
setRectTransformAnchorMinReq :: KRPCHS.UI.RectTransform -> (Double, Double) -> KRPCCallReq ()
setRectTransformAnchorMinReq thisArg valueArg = makeCallReq "UI" "RectTransform_set_AnchorMin" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setRectTransformAnchorMin :: KRPCHS.UI.RectTransform -> (Double, Double) -> RPCContext ()
setRectTransformAnchorMin thisArg valueArg = simpleRequest $ setRectTransformAnchorMinReq thisArg valueArg 

{-|
Position of the rectangles pivot point relative to the anchors.
 -}
setRectTransformLocalPositionReq :: KRPCHS.UI.RectTransform -> (Double, Double, Double) -> KRPCCallReq ()
setRectTransformLocalPositionReq thisArg valueArg = makeCallReq "UI" "RectTransform_set_LocalPosition" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setRectTransformLocalPosition :: KRPCHS.UI.RectTransform -> (Double, Double, Double) -> RPCContext ()
setRectTransformLocalPosition thisArg valueArg = simpleRequest $ setRectTransformLocalPositionReq thisArg valueArg 

{-|
Position of the rectangles lower left corner relative to the anchors.
 -}
setRectTransformLowerLeftReq :: KRPCHS.UI.RectTransform -> (Double, Double) -> KRPCCallReq ()
setRectTransformLowerLeftReq thisArg valueArg = makeCallReq "UI" "RectTransform_set_LowerLeft" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setRectTransformLowerLeft :: KRPCHS.UI.RectTransform -> (Double, Double) -> RPCContext ()
setRectTransformLowerLeft thisArg valueArg = simpleRequest $ setRectTransformLowerLeftReq thisArg valueArg 

{-|
Location of the pivot point around which the rectangle rotates, defined as a fraction of the size of the rectangle itself.
 -}
setRectTransformPivotReq :: KRPCHS.UI.RectTransform -> (Double, Double) -> KRPCCallReq ()
setRectTransformPivotReq thisArg valueArg = makeCallReq "UI" "RectTransform_set_Pivot" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setRectTransformPivot :: KRPCHS.UI.RectTransform -> (Double, Double) -> RPCContext ()
setRectTransformPivot thisArg valueArg = simpleRequest $ setRectTransformPivotReq thisArg valueArg 

{-|
Position of the rectangles pivot point relative to the anchors.
 -}
setRectTransformPositionReq :: KRPCHS.UI.RectTransform -> (Double, Double) -> KRPCCallReq ()
setRectTransformPositionReq thisArg valueArg = makeCallReq "UI" "RectTransform_set_Position" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setRectTransformPosition :: KRPCHS.UI.RectTransform -> (Double, Double) -> RPCContext ()
setRectTransformPosition thisArg valueArg = simpleRequest $ setRectTransformPositionReq thisArg valueArg 

{-|
Rotation, as a quaternion, of the object around its pivot point.
 -}
setRectTransformRotationReq :: KRPCHS.UI.RectTransform -> (Double, Double, Double, Double) -> KRPCCallReq ()
setRectTransformRotationReq thisArg valueArg = makeCallReq "UI" "RectTransform_set_Rotation" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setRectTransformRotation :: KRPCHS.UI.RectTransform -> (Double, Double, Double, Double) -> RPCContext ()
setRectTransformRotation thisArg valueArg = simpleRequest $ setRectTransformRotationReq thisArg valueArg 

{-|
Scale factor applied to the object in the x, y and z dimensions.
 -}
setRectTransformScaleReq :: KRPCHS.UI.RectTransform -> (Double, Double, Double) -> KRPCCallReq ()
setRectTransformScaleReq thisArg valueArg = makeCallReq "UI" "RectTransform_set_Scale" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setRectTransformScale :: KRPCHS.UI.RectTransform -> (Double, Double, Double) -> RPCContext ()
setRectTransformScale thisArg valueArg = simpleRequest $ setRectTransformScaleReq thisArg valueArg 

{-|
Width and height of the rectangle.
 -}
setRectTransformSizeReq :: KRPCHS.UI.RectTransform -> (Double, Double) -> KRPCCallReq ()
setRectTransformSizeReq thisArg valueArg = makeCallReq "UI" "RectTransform_set_Size" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setRectTransformSize :: KRPCHS.UI.RectTransform -> (Double, Double) -> RPCContext ()
setRectTransformSize thisArg valueArg = simpleRequest $ setRectTransformSizeReq thisArg valueArg 

{-|
Position of the rectangles upper right corner relative to the anchors.
 -}
setRectTransformUpperRightReq :: KRPCHS.UI.RectTransform -> (Double, Double) -> KRPCCallReq ()
setRectTransformUpperRightReq thisArg valueArg = makeCallReq "UI" "RectTransform_set_UpperRight" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setRectTransformUpperRight :: KRPCHS.UI.RectTransform -> (Double, Double) -> RPCContext ()
setRectTransformUpperRight thisArg valueArg = simpleRequest $ setRectTransformUpperRightReq thisArg valueArg 

{-|
Remove the UI object.
 -}
textRemoveReq :: KRPCHS.UI.Text -> KRPCCallReq ()
textRemoveReq thisArg = makeCallReq "UI" "Text_Remove" [makeArgument 0 thisArg]

textRemove :: KRPCHS.UI.Text -> RPCContext ()
textRemove thisArg = simpleRequest $ textRemoveReq thisArg 

{-|
Alignment.
 -}
getTextAlignmentReq :: KRPCHS.UI.Text -> KRPCCallReq (KRPCHS.UI.TextAnchor)
getTextAlignmentReq thisArg = makeCallReq "UI" "Text_get_Alignment" [makeArgument 0 thisArg]

getTextAlignment :: KRPCHS.UI.Text -> RPCContext (KRPCHS.UI.TextAnchor)
getTextAlignment thisArg = simpleRequest $ getTextAlignmentReq thisArg

getTextAlignmentStreamReq :: KRPCHS.UI.Text -> KRPCStreamReq (KRPCHS.UI.TextAnchor)
getTextAlignmentStreamReq thisArg = makeStreamReq $ getTextAlignmentReq thisArg

getTextAlignmentStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream (KRPCHS.UI.TextAnchor))
getTextAlignmentStream thisArg = requestAddStream $ getTextAlignmentStreamReq thisArg 

{-|
A list of all available fonts.
 -}
getTextAvailableFontsReq :: KRPCHS.UI.Text -> KRPCCallReq ([Data.Text.Text])
getTextAvailableFontsReq thisArg = makeCallReq "UI" "Text_get_AvailableFonts" [makeArgument 0 thisArg]

getTextAvailableFonts :: KRPCHS.UI.Text -> RPCContext ([Data.Text.Text])
getTextAvailableFonts thisArg = simpleRequest $ getTextAvailableFontsReq thisArg

getTextAvailableFontsStreamReq :: KRPCHS.UI.Text -> KRPCStreamReq ([Data.Text.Text])
getTextAvailableFontsStreamReq thisArg = makeStreamReq $ getTextAvailableFontsReq thisArg

getTextAvailableFontsStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream ([Data.Text.Text]))
getTextAvailableFontsStream thisArg = requestAddStream $ getTextAvailableFontsStreamReq thisArg 

{-|
Set the color
 -}
getTextColorReq :: KRPCHS.UI.Text -> KRPCCallReq ((Double, Double, Double))
getTextColorReq thisArg = makeCallReq "UI" "Text_get_Color" [makeArgument 0 thisArg]

getTextColor :: KRPCHS.UI.Text -> RPCContext ((Double, Double, Double))
getTextColor thisArg = simpleRequest $ getTextColorReq thisArg

getTextColorStreamReq :: KRPCHS.UI.Text -> KRPCStreamReq ((Double, Double, Double))
getTextColorStreamReq thisArg = makeStreamReq $ getTextColorReq thisArg

getTextColorStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream ((Double, Double, Double)))
getTextColorStream thisArg = requestAddStream $ getTextColorStreamReq thisArg 

{-|
The text string
 -}
getTextContentReq :: KRPCHS.UI.Text -> KRPCCallReq (Data.Text.Text)
getTextContentReq thisArg = makeCallReq "UI" "Text_get_Content" [makeArgument 0 thisArg]

getTextContent :: KRPCHS.UI.Text -> RPCContext (Data.Text.Text)
getTextContent thisArg = simpleRequest $ getTextContentReq thisArg

getTextContentStreamReq :: KRPCHS.UI.Text -> KRPCStreamReq (Data.Text.Text)
getTextContentStreamReq thisArg = makeStreamReq $ getTextContentReq thisArg

getTextContentStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream (Data.Text.Text))
getTextContentStream thisArg = requestAddStream $ getTextContentStreamReq thisArg 

{-|
Name of the font
 -}
getTextFontReq :: KRPCHS.UI.Text -> KRPCCallReq (Data.Text.Text)
getTextFontReq thisArg = makeCallReq "UI" "Text_get_Font" [makeArgument 0 thisArg]

getTextFont :: KRPCHS.UI.Text -> RPCContext (Data.Text.Text)
getTextFont thisArg = simpleRequest $ getTextFontReq thisArg

getTextFontStreamReq :: KRPCHS.UI.Text -> KRPCStreamReq (Data.Text.Text)
getTextFontStreamReq thisArg = makeStreamReq $ getTextFontReq thisArg

getTextFontStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream (Data.Text.Text))
getTextFontStream thisArg = requestAddStream $ getTextFontStreamReq thisArg 

{-|
Line spacing.
 -}
getTextLineSpacingReq :: KRPCHS.UI.Text -> KRPCCallReq (Float)
getTextLineSpacingReq thisArg = makeCallReq "UI" "Text_get_LineSpacing" [makeArgument 0 thisArg]

getTextLineSpacing :: KRPCHS.UI.Text -> RPCContext (Float)
getTextLineSpacing thisArg = simpleRequest $ getTextLineSpacingReq thisArg

getTextLineSpacingStreamReq :: KRPCHS.UI.Text -> KRPCStreamReq (Float)
getTextLineSpacingStreamReq thisArg = makeStreamReq $ getTextLineSpacingReq thisArg

getTextLineSpacingStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream (Float))
getTextLineSpacingStream thisArg = requestAddStream $ getTextLineSpacingStreamReq thisArg 

{-|
The rect transform for the text.
 -}
getTextRectTransformReq :: KRPCHS.UI.Text -> KRPCCallReq (KRPCHS.UI.RectTransform)
getTextRectTransformReq thisArg = makeCallReq "UI" "Text_get_RectTransform" [makeArgument 0 thisArg]

getTextRectTransform :: KRPCHS.UI.Text -> RPCContext (KRPCHS.UI.RectTransform)
getTextRectTransform thisArg = simpleRequest $ getTextRectTransformReq thisArg

getTextRectTransformStreamReq :: KRPCHS.UI.Text -> KRPCStreamReq (KRPCHS.UI.RectTransform)
getTextRectTransformStreamReq thisArg = makeStreamReq $ getTextRectTransformReq thisArg

getTextRectTransformStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream (KRPCHS.UI.RectTransform))
getTextRectTransformStream thisArg = requestAddStream $ getTextRectTransformStreamReq thisArg 

{-|
Font size.
 -}
getTextSizeReq :: KRPCHS.UI.Text -> KRPCCallReq (Data.Int.Int32)
getTextSizeReq thisArg = makeCallReq "UI" "Text_get_Size" [makeArgument 0 thisArg]

getTextSize :: KRPCHS.UI.Text -> RPCContext (Data.Int.Int32)
getTextSize thisArg = simpleRequest $ getTextSizeReq thisArg

getTextSizeStreamReq :: KRPCHS.UI.Text -> KRPCStreamReq (Data.Int.Int32)
getTextSizeStreamReq thisArg = makeStreamReq $ getTextSizeReq thisArg

getTextSizeStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream (Data.Int.Int32))
getTextSizeStream thisArg = requestAddStream $ getTextSizeStreamReq thisArg 

{-|
Font style.
 -}
getTextStyleReq :: KRPCHS.UI.Text -> KRPCCallReq (KRPCHS.UI.FontStyle)
getTextStyleReq thisArg = makeCallReq "UI" "Text_get_Style" [makeArgument 0 thisArg]

getTextStyle :: KRPCHS.UI.Text -> RPCContext (KRPCHS.UI.FontStyle)
getTextStyle thisArg = simpleRequest $ getTextStyleReq thisArg

getTextStyleStreamReq :: KRPCHS.UI.Text -> KRPCStreamReq (KRPCHS.UI.FontStyle)
getTextStyleStreamReq thisArg = makeStreamReq $ getTextStyleReq thisArg

getTextStyleStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream (KRPCHS.UI.FontStyle))
getTextStyleStream thisArg = requestAddStream $ getTextStyleStreamReq thisArg 

{-|
Whether the UI object is visible.
 -}
getTextVisibleReq :: KRPCHS.UI.Text -> KRPCCallReq (Bool)
getTextVisibleReq thisArg = makeCallReq "UI" "Text_get_Visible" [makeArgument 0 thisArg]

getTextVisible :: KRPCHS.UI.Text -> RPCContext (Bool)
getTextVisible thisArg = simpleRequest $ getTextVisibleReq thisArg

getTextVisibleStreamReq :: KRPCHS.UI.Text -> KRPCStreamReq (Bool)
getTextVisibleStreamReq thisArg = makeStreamReq $ getTextVisibleReq thisArg

getTextVisibleStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream (Bool))
getTextVisibleStream thisArg = requestAddStream $ getTextVisibleStreamReq thisArg 

{-|
Alignment.
 -}
setTextAlignmentReq :: KRPCHS.UI.Text -> KRPCHS.UI.TextAnchor -> KRPCCallReq ()
setTextAlignmentReq thisArg valueArg = makeCallReq "UI" "Text_set_Alignment" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setTextAlignment :: KRPCHS.UI.Text -> KRPCHS.UI.TextAnchor -> RPCContext ()
setTextAlignment thisArg valueArg = simpleRequest $ setTextAlignmentReq thisArg valueArg 

{-|
Set the color
 -}
setTextColorReq :: KRPCHS.UI.Text -> (Double, Double, Double) -> KRPCCallReq ()
setTextColorReq thisArg valueArg = makeCallReq "UI" "Text_set_Color" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setTextColor :: KRPCHS.UI.Text -> (Double, Double, Double) -> RPCContext ()
setTextColor thisArg valueArg = simpleRequest $ setTextColorReq thisArg valueArg 

{-|
The text string
 -}
setTextContentReq :: KRPCHS.UI.Text -> Data.Text.Text -> KRPCCallReq ()
setTextContentReq thisArg valueArg = makeCallReq "UI" "Text_set_Content" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setTextContent :: KRPCHS.UI.Text -> Data.Text.Text -> RPCContext ()
setTextContent thisArg valueArg = simpleRequest $ setTextContentReq thisArg valueArg 

{-|
Name of the font
 -}
setTextFontReq :: KRPCHS.UI.Text -> Data.Text.Text -> KRPCCallReq ()
setTextFontReq thisArg valueArg = makeCallReq "UI" "Text_set_Font" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setTextFont :: KRPCHS.UI.Text -> Data.Text.Text -> RPCContext ()
setTextFont thisArg valueArg = simpleRequest $ setTextFontReq thisArg valueArg 

{-|
Line spacing.
 -}
setTextLineSpacingReq :: KRPCHS.UI.Text -> Float -> KRPCCallReq ()
setTextLineSpacingReq thisArg valueArg = makeCallReq "UI" "Text_set_LineSpacing" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setTextLineSpacing :: KRPCHS.UI.Text -> Float -> RPCContext ()
setTextLineSpacing thisArg valueArg = simpleRequest $ setTextLineSpacingReq thisArg valueArg 

{-|
Font size.
 -}
setTextSizeReq :: KRPCHS.UI.Text -> Data.Int.Int32 -> KRPCCallReq ()
setTextSizeReq thisArg valueArg = makeCallReq "UI" "Text_set_Size" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setTextSize :: KRPCHS.UI.Text -> Data.Int.Int32 -> RPCContext ()
setTextSize thisArg valueArg = simpleRequest $ setTextSizeReq thisArg valueArg 

{-|
Font style.
 -}
setTextStyleReq :: KRPCHS.UI.Text -> KRPCHS.UI.FontStyle -> KRPCCallReq ()
setTextStyleReq thisArg valueArg = makeCallReq "UI" "Text_set_Style" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setTextStyle :: KRPCHS.UI.Text -> KRPCHS.UI.FontStyle -> RPCContext ()
setTextStyle thisArg valueArg = simpleRequest $ setTextStyleReq thisArg valueArg 

{-|
Whether the UI object is visible.
 -}
setTextVisibleReq :: KRPCHS.UI.Text -> Bool -> KRPCCallReq ()
setTextVisibleReq thisArg valueArg = makeCallReq "UI" "Text_set_Visible" [makeArgument 0 thisArg, makeArgument 1 valueArg]

setTextVisible :: KRPCHS.UI.Text -> Bool -> RPCContext ()
setTextVisible thisArg valueArg = simpleRequest $ setTextVisibleReq thisArg valueArg 

{-|
The stock UI canvas.
 -}
getStockCanvasReq :: KRPCCallReq (KRPCHS.UI.Canvas)
getStockCanvasReq  = makeCallReq "UI" "get_StockCanvas" []

getStockCanvas :: RPCContext (KRPCHS.UI.Canvas)
getStockCanvas  = simpleRequest $ getStockCanvasReq 

getStockCanvasStreamReq :: KRPCStreamReq (KRPCHS.UI.Canvas)
getStockCanvasStreamReq  = makeStreamReq $ getStockCanvasReq 

getStockCanvasStream :: RPCContext (KRPCStream (KRPCHS.UI.Canvas))
getStockCanvasStream  = requestAddStream $ getStockCanvasStreamReq  

