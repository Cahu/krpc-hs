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
, addCanvasStream
, addCanvasStreamReq
, buttonRemove
, getButtonClicked
, getButtonClickedStream
, getButtonClickedStreamReq
, getButtonRectTransform
, getButtonRectTransformStream
, getButtonRectTransformStreamReq
, getButtonText
, getButtonTextStream
, getButtonTextStreamReq
, getButtonVisible
, getButtonVisibleStream
, getButtonVisibleStreamReq
, setButtonClicked
, setButtonVisible
, canvasAddButton
, canvasAddButtonStream
, canvasAddButtonStreamReq
, canvasAddInputField
, canvasAddInputFieldStream
, canvasAddInputFieldStreamReq
, canvasAddPanel
, canvasAddPanelStream
, canvasAddPanelStreamReq
, canvasAddText
, canvasAddTextStream
, canvasAddTextStreamReq
, canvasRemove
, getCanvasRectTransform
, getCanvasRectTransformStream
, getCanvasRectTransformStreamReq
, getCanvasVisible
, getCanvasVisibleStream
, getCanvasVisibleStreamReq
, setCanvasVisible
, clear
, inputFieldRemove
, getInputFieldChanged
, getInputFieldChangedStream
, getInputFieldChangedStreamReq
, getInputFieldRectTransform
, getInputFieldRectTransformStream
, getInputFieldRectTransformStreamReq
, getInputFieldText
, getInputFieldTextStream
, getInputFieldTextStreamReq
, getInputFieldValue
, getInputFieldValueStream
, getInputFieldValueStreamReq
, getInputFieldVisible
, getInputFieldVisibleStream
, getInputFieldVisibleStreamReq
, setInputFieldChanged
, setInputFieldValue
, setInputFieldVisible
, message
, panelAddButton
, panelAddButtonStream
, panelAddButtonStreamReq
, panelAddInputField
, panelAddInputFieldStream
, panelAddInputFieldStreamReq
, panelAddPanel
, panelAddPanelStream
, panelAddPanelStreamReq
, panelAddText
, panelAddTextStream
, panelAddTextStreamReq
, panelRemove
, getPanelRectTransform
, getPanelRectTransformStream
, getPanelRectTransformStreamReq
, getPanelVisible
, getPanelVisibleStream
, getPanelVisibleStreamReq
, setPanelVisible
, getRectTransformAnchorMax
, getRectTransformAnchorMaxStream
, getRectTransformAnchorMaxStreamReq
, getRectTransformAnchorMin
, getRectTransformAnchorMinStream
, getRectTransformAnchorMinStreamReq
, getRectTransformLocalPosition
, getRectTransformLocalPositionStream
, getRectTransformLocalPositionStreamReq
, getRectTransformLowerLeft
, getRectTransformLowerLeftStream
, getRectTransformLowerLeftStreamReq
, getRectTransformPivot
, getRectTransformPivotStream
, getRectTransformPivotStreamReq
, getRectTransformPosition
, getRectTransformPositionStream
, getRectTransformPositionStreamReq
, getRectTransformRotation
, getRectTransformRotationStream
, getRectTransformRotationStreamReq
, getRectTransformScale
, getRectTransformScaleStream
, getRectTransformScaleStreamReq
, getRectTransformSize
, getRectTransformSizeStream
, getRectTransformSizeStreamReq
, getRectTransformUpperRight
, getRectTransformUpperRightStream
, getRectTransformUpperRightStreamReq
, setRectTransformAnchor
, setRectTransformAnchorMax
, setRectTransformAnchorMin
, setRectTransformLocalPosition
, setRectTransformLowerLeft
, setRectTransformPivot
, setRectTransformPosition
, setRectTransformRotation
, setRectTransformScale
, setRectTransformSize
, setRectTransformUpperRight
, textRemove
, getTextAlignment
, getTextAlignmentStream
, getTextAlignmentStreamReq
, getTextAvailableFonts
, getTextAvailableFontsStream
, getTextAvailableFontsStreamReq
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
, getTextRectTransform
, getTextRectTransformStream
, getTextRectTransformStreamReq
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
, setTextColor
, setTextContent
, setTextFont
, setTextLineSpacing
, setTextSize
, setTextStyle
, setTextVisible
, getStockCanvas
, getStockCanvasStream
, getStockCanvasStreamReq
) where

import qualified Data.Int
import qualified Data.Text

import KRPCHS.Internal.Requests
import KRPCHS.Internal.SerializeUtils


{-
 - A text label. See <see cref="M:UI.Panel.AddButton" />.
 -}
newtype Button = Button { buttonId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Button where
    encodePb   = encodePb . buttonId
    decodePb b = Button <$> decodePb b

instance KRPCResponseExtractable Button

{-
 - A canvas for user interface elements. See <see cref="M:UI.StockCanvas" /> and <see cref="M:UI.AddCanvas" />.
 -}
newtype Canvas = Canvas { canvasId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Canvas where
    encodePb   = encodePb . canvasId
    decodePb b = Canvas <$> decodePb b

instance KRPCResponseExtractable Canvas

{-
 - An input field. See <see cref="M:UI.Panel.AddInputField" />.
 -}
newtype InputField = InputField { inputFieldId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable InputField where
    encodePb   = encodePb . inputFieldId
    decodePb b = InputField <$> decodePb b

instance KRPCResponseExtractable InputField

{-
 - A container for user interface elements. See <see cref="M:UI.Canvas.AddPanel" />.
 -}
newtype Panel = Panel { panelId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Panel where
    encodePb   = encodePb . panelId
    decodePb b = Panel <$> decodePb b

instance KRPCResponseExtractable Panel

{-
 - A Unity engine Rect Transform for a UI object.
 - See the <a href="http://docs.unity3d.com/Manual/class-RectTransform.html">Unity manualfor more details.
 -}
newtype RectTransform = RectTransform { rectTransformId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable RectTransform where
    encodePb   = encodePb . rectTransformId
    decodePb b = RectTransform <$> decodePb b

instance KRPCResponseExtractable RectTransform

{-
 - A text label. See <see cref="M:UI.Panel.AddText" />.
 -}
newtype Text = Text { textId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable Text where
    encodePb   = encodePb . textId
    decodePb b = Text <$> decodePb b

instance KRPCResponseExtractable Text


{-
 - Font style.
 -}
data FontStyle
    = FontStyle'Normal
    | FontStyle'Bold
    | FontStyle'Italic
    | FontStyle'BoldAndItalic
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable FontStyle where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable FontStyle

{-
 - Message position.
 -}
data MessagePosition
    = MessagePosition'BottomCenter
    | MessagePosition'TopCenter
    | MessagePosition'TopLeft
    | MessagePosition'TopRight
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable MessagePosition where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable MessagePosition

{-
 - Text alignment.
 -}
data TextAlignment
    = TextAlignment'Left
    | TextAlignment'Right
    | TextAlignment'Center
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable TextAlignment where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable TextAlignment

{-
 - Text alignment.
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

instance PbSerializable TextAnchor where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable TextAnchor


{-
 - Add a new canvas.If you want to add UI elements to KSPs stock UI canvas, use <see cref="M:UI.StockCanvas" />.
 -}
addCanvas :: RPCContext (KRPCHS.UI.Canvas)
addCanvas  = do
    let r = makeRequest "UI" "AddCanvas" []
    res <- sendRequest r
    processResponse res

addCanvasStreamReq :: RPCContext (KRPCStreamReq (KRPCHS.UI.Canvas))
addCanvasStreamReq  = do
    let req = makeRequest "UI" "AddCanvas" []
    return (makeStream req)

addCanvasStream :: RPCContext (KRPCStream (KRPCHS.UI.Canvas))
addCanvasStream  = requestStream =<< addCanvasStreamReq  

{-
 - Remove the UI object.
 -}
buttonRemove :: KRPCHS.UI.Button -> RPCContext ()
buttonRemove thisArg = do
    let r = makeRequest "UI" "Button_Remove" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the button has been clicked.This property is set to true when the user clicks the button.
 - A client script should reset the property to false in order to detect subsequent button presses.
 -}
getButtonClicked :: KRPCHS.UI.Button -> RPCContext (Bool)
getButtonClicked thisArg = do
    let r = makeRequest "UI" "Button_get_Clicked" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getButtonClickedStreamReq :: KRPCHS.UI.Button -> RPCContext (KRPCStreamReq (Bool))
getButtonClickedStreamReq thisArg = do
    let req = makeRequest "UI" "Button_get_Clicked" [makeArgument 0 thisArg]
    return (makeStream req)

getButtonClickedStream :: KRPCHS.UI.Button -> RPCContext (KRPCStream (Bool))
getButtonClickedStream thisArg = requestStream =<< getButtonClickedStreamReq thisArg 

{-
 - The rect transform for the text.
 -}
getButtonRectTransform :: KRPCHS.UI.Button -> RPCContext (KRPCHS.UI.RectTransform)
getButtonRectTransform thisArg = do
    let r = makeRequest "UI" "Button_get_RectTransform" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getButtonRectTransformStreamReq :: KRPCHS.UI.Button -> RPCContext (KRPCStreamReq (KRPCHS.UI.RectTransform))
getButtonRectTransformStreamReq thisArg = do
    let req = makeRequest "UI" "Button_get_RectTransform" [makeArgument 0 thisArg]
    return (makeStream req)

getButtonRectTransformStream :: KRPCHS.UI.Button -> RPCContext (KRPCStream (KRPCHS.UI.RectTransform))
getButtonRectTransformStream thisArg = requestStream =<< getButtonRectTransformStreamReq thisArg 

{-
 - The text for the button.
 -}
getButtonText :: KRPCHS.UI.Button -> RPCContext (KRPCHS.UI.Text)
getButtonText thisArg = do
    let r = makeRequest "UI" "Button_get_Text" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getButtonTextStreamReq :: KRPCHS.UI.Button -> RPCContext (KRPCStreamReq (KRPCHS.UI.Text))
getButtonTextStreamReq thisArg = do
    let req = makeRequest "UI" "Button_get_Text" [makeArgument 0 thisArg]
    return (makeStream req)

getButtonTextStream :: KRPCHS.UI.Button -> RPCContext (KRPCStream (KRPCHS.UI.Text))
getButtonTextStream thisArg = requestStream =<< getButtonTextStreamReq thisArg 

{-
 - Whether the UI object is visible.
 -}
getButtonVisible :: KRPCHS.UI.Button -> RPCContext (Bool)
getButtonVisible thisArg = do
    let r = makeRequest "UI" "Button_get_Visible" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getButtonVisibleStreamReq :: KRPCHS.UI.Button -> RPCContext (KRPCStreamReq (Bool))
getButtonVisibleStreamReq thisArg = do
    let req = makeRequest "UI" "Button_get_Visible" [makeArgument 0 thisArg]
    return (makeStream req)

getButtonVisibleStream :: KRPCHS.UI.Button -> RPCContext (KRPCStream (Bool))
getButtonVisibleStream thisArg = requestStream =<< getButtonVisibleStreamReq thisArg 

{-
 - Whether the button has been clicked.This property is set to true when the user clicks the button.
 - A client script should reset the property to false in order to detect subsequent button presses.
 -}
setButtonClicked :: KRPCHS.UI.Button -> Bool -> RPCContext ()
setButtonClicked thisArg valueArg = do
    let r = makeRequest "UI" "Button_set_Clicked" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the UI object is visible.
 -}
setButtonVisible :: KRPCHS.UI.Button -> Bool -> RPCContext ()
setButtonVisible thisArg valueArg = do
    let r = makeRequest "UI" "Button_set_Visible" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Add a button to the canvas.<param name="content">The label for the button.<param name="visible">Whether the button is visible.
 -}
canvasAddButton :: KRPCHS.UI.Canvas -> Data.Text.Text -> Bool -> RPCContext (KRPCHS.UI.Button)
canvasAddButton thisArg contentArg visibleArg = do
    let r = makeRequest "UI" "Canvas_AddButton" [makeArgument 0 thisArg, makeArgument 1 contentArg, makeArgument 2 visibleArg]
    res <- sendRequest r
    processResponse res

canvasAddButtonStreamReq :: KRPCHS.UI.Canvas -> Data.Text.Text -> Bool -> RPCContext (KRPCStreamReq (KRPCHS.UI.Button))
canvasAddButtonStreamReq thisArg contentArg visibleArg = do
    let req = makeRequest "UI" "Canvas_AddButton" [makeArgument 0 thisArg, makeArgument 1 contentArg, makeArgument 2 visibleArg]
    return (makeStream req)

canvasAddButtonStream :: KRPCHS.UI.Canvas -> Data.Text.Text -> Bool -> RPCContext (KRPCStream (KRPCHS.UI.Button))
canvasAddButtonStream thisArg contentArg visibleArg = requestStream =<< canvasAddButtonStreamReq thisArg contentArg visibleArg 

{-
 - Add an input field to the canvas.<param name="visible">Whether the input field is visible.
 -}
canvasAddInputField :: KRPCHS.UI.Canvas -> Bool -> RPCContext (KRPCHS.UI.InputField)
canvasAddInputField thisArg visibleArg = do
    let r = makeRequest "UI" "Canvas_AddInputField" [makeArgument 0 thisArg, makeArgument 1 visibleArg]
    res <- sendRequest r
    processResponse res

canvasAddInputFieldStreamReq :: KRPCHS.UI.Canvas -> Bool -> RPCContext (KRPCStreamReq (KRPCHS.UI.InputField))
canvasAddInputFieldStreamReq thisArg visibleArg = do
    let req = makeRequest "UI" "Canvas_AddInputField" [makeArgument 0 thisArg, makeArgument 1 visibleArg]
    return (makeStream req)

canvasAddInputFieldStream :: KRPCHS.UI.Canvas -> Bool -> RPCContext (KRPCStream (KRPCHS.UI.InputField))
canvasAddInputFieldStream thisArg visibleArg = requestStream =<< canvasAddInputFieldStreamReq thisArg visibleArg 

{-
 - Create a new container for user interface elements.<param name="visible">Whether the panel is visible.
 -}
canvasAddPanel :: KRPCHS.UI.Canvas -> Bool -> RPCContext (KRPCHS.UI.Panel)
canvasAddPanel thisArg visibleArg = do
    let r = makeRequest "UI" "Canvas_AddPanel" [makeArgument 0 thisArg, makeArgument 1 visibleArg]
    res <- sendRequest r
    processResponse res

canvasAddPanelStreamReq :: KRPCHS.UI.Canvas -> Bool -> RPCContext (KRPCStreamReq (KRPCHS.UI.Panel))
canvasAddPanelStreamReq thisArg visibleArg = do
    let req = makeRequest "UI" "Canvas_AddPanel" [makeArgument 0 thisArg, makeArgument 1 visibleArg]
    return (makeStream req)

canvasAddPanelStream :: KRPCHS.UI.Canvas -> Bool -> RPCContext (KRPCStream (KRPCHS.UI.Panel))
canvasAddPanelStream thisArg visibleArg = requestStream =<< canvasAddPanelStreamReq thisArg visibleArg 

{-
 - Add text to the canvas.<param name="content">The text.<param name="visible">Whether the text is visible.
 -}
canvasAddText :: KRPCHS.UI.Canvas -> Data.Text.Text -> Bool -> RPCContext (KRPCHS.UI.Text)
canvasAddText thisArg contentArg visibleArg = do
    let r = makeRequest "UI" "Canvas_AddText" [makeArgument 0 thisArg, makeArgument 1 contentArg, makeArgument 2 visibleArg]
    res <- sendRequest r
    processResponse res

canvasAddTextStreamReq :: KRPCHS.UI.Canvas -> Data.Text.Text -> Bool -> RPCContext (KRPCStreamReq (KRPCHS.UI.Text))
canvasAddTextStreamReq thisArg contentArg visibleArg = do
    let req = makeRequest "UI" "Canvas_AddText" [makeArgument 0 thisArg, makeArgument 1 contentArg, makeArgument 2 visibleArg]
    return (makeStream req)

canvasAddTextStream :: KRPCHS.UI.Canvas -> Data.Text.Text -> Bool -> RPCContext (KRPCStream (KRPCHS.UI.Text))
canvasAddTextStream thisArg contentArg visibleArg = requestStream =<< canvasAddTextStreamReq thisArg contentArg visibleArg 

{-
 - Remove the UI object.
 -}
canvasRemove :: KRPCHS.UI.Canvas -> RPCContext ()
canvasRemove thisArg = do
    let r = makeRequest "UI" "Canvas_Remove" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - The rect transform for the canvas.
 -}
getCanvasRectTransform :: KRPCHS.UI.Canvas -> RPCContext (KRPCHS.UI.RectTransform)
getCanvasRectTransform thisArg = do
    let r = makeRequest "UI" "Canvas_get_RectTransform" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCanvasRectTransformStreamReq :: KRPCHS.UI.Canvas -> RPCContext (KRPCStreamReq (KRPCHS.UI.RectTransform))
getCanvasRectTransformStreamReq thisArg = do
    let req = makeRequest "UI" "Canvas_get_RectTransform" [makeArgument 0 thisArg]
    return (makeStream req)

getCanvasRectTransformStream :: KRPCHS.UI.Canvas -> RPCContext (KRPCStream (KRPCHS.UI.RectTransform))
getCanvasRectTransformStream thisArg = requestStream =<< getCanvasRectTransformStreamReq thisArg 

{-
 - Whether the UI object is visible.
 -}
getCanvasVisible :: KRPCHS.UI.Canvas -> RPCContext (Bool)
getCanvasVisible thisArg = do
    let r = makeRequest "UI" "Canvas_get_Visible" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getCanvasVisibleStreamReq :: KRPCHS.UI.Canvas -> RPCContext (KRPCStreamReq (Bool))
getCanvasVisibleStreamReq thisArg = do
    let req = makeRequest "UI" "Canvas_get_Visible" [makeArgument 0 thisArg]
    return (makeStream req)

getCanvasVisibleStream :: KRPCHS.UI.Canvas -> RPCContext (KRPCStream (Bool))
getCanvasVisibleStream thisArg = requestStream =<< getCanvasVisibleStreamReq thisArg 

{-
 - Whether the UI object is visible.
 -}
setCanvasVisible :: KRPCHS.UI.Canvas -> Bool -> RPCContext ()
setCanvasVisible thisArg valueArg = do
    let r = makeRequest "UI" "Canvas_set_Visible" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Remove all user interface elements.<param name="clientOnly">If true, only remove objects created by the calling client.
 -}
clear :: Bool -> RPCContext ()
clear clientOnlyArg = do
    let r = makeRequest "UI" "Clear" [makeArgument 0 clientOnlyArg]
    res <- sendRequest r
    processResponse res 

{-
 - Remove the UI object.
 -}
inputFieldRemove :: KRPCHS.UI.InputField -> RPCContext ()
inputFieldRemove thisArg = do
    let r = makeRequest "UI" "InputField_Remove" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the input field has been changed.This property is set to true when the user modifies the value of the input field.
 - A client script should reset the property to false in order to detect subsequent changes.
 -}
getInputFieldChanged :: KRPCHS.UI.InputField -> RPCContext (Bool)
getInputFieldChanged thisArg = do
    let r = makeRequest "UI" "InputField_get_Changed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getInputFieldChangedStreamReq :: KRPCHS.UI.InputField -> RPCContext (KRPCStreamReq (Bool))
getInputFieldChangedStreamReq thisArg = do
    let req = makeRequest "UI" "InputField_get_Changed" [makeArgument 0 thisArg]
    return (makeStream req)

getInputFieldChangedStream :: KRPCHS.UI.InputField -> RPCContext (KRPCStream (Bool))
getInputFieldChangedStream thisArg = requestStream =<< getInputFieldChangedStreamReq thisArg 

{-
 - The rect transform for the input field.
 -}
getInputFieldRectTransform :: KRPCHS.UI.InputField -> RPCContext (KRPCHS.UI.RectTransform)
getInputFieldRectTransform thisArg = do
    let r = makeRequest "UI" "InputField_get_RectTransform" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getInputFieldRectTransformStreamReq :: KRPCHS.UI.InputField -> RPCContext (KRPCStreamReq (KRPCHS.UI.RectTransform))
getInputFieldRectTransformStreamReq thisArg = do
    let req = makeRequest "UI" "InputField_get_RectTransform" [makeArgument 0 thisArg]
    return (makeStream req)

getInputFieldRectTransformStream :: KRPCHS.UI.InputField -> RPCContext (KRPCStream (KRPCHS.UI.RectTransform))
getInputFieldRectTransformStream thisArg = requestStream =<< getInputFieldRectTransformStreamReq thisArg 

{-
 - The text component of the input field.Use <see cref="M:UI.InputField.Value" /> to get and set the value in the field.
 - This object can be used to alter the style of the input field's text.
 -}
getInputFieldText :: KRPCHS.UI.InputField -> RPCContext (KRPCHS.UI.Text)
getInputFieldText thisArg = do
    let r = makeRequest "UI" "InputField_get_Text" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getInputFieldTextStreamReq :: KRPCHS.UI.InputField -> RPCContext (KRPCStreamReq (KRPCHS.UI.Text))
getInputFieldTextStreamReq thisArg = do
    let req = makeRequest "UI" "InputField_get_Text" [makeArgument 0 thisArg]
    return (makeStream req)

getInputFieldTextStream :: KRPCHS.UI.InputField -> RPCContext (KRPCStream (KRPCHS.UI.Text))
getInputFieldTextStream thisArg = requestStream =<< getInputFieldTextStreamReq thisArg 

{-
 - The value of the input field.
 -}
getInputFieldValue :: KRPCHS.UI.InputField -> RPCContext (Data.Text.Text)
getInputFieldValue thisArg = do
    let r = makeRequest "UI" "InputField_get_Value" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getInputFieldValueStreamReq :: KRPCHS.UI.InputField -> RPCContext (KRPCStreamReq (Data.Text.Text))
getInputFieldValueStreamReq thisArg = do
    let req = makeRequest "UI" "InputField_get_Value" [makeArgument 0 thisArg]
    return (makeStream req)

getInputFieldValueStream :: KRPCHS.UI.InputField -> RPCContext (KRPCStream (Data.Text.Text))
getInputFieldValueStream thisArg = requestStream =<< getInputFieldValueStreamReq thisArg 

{-
 - Whether the UI object is visible.
 -}
getInputFieldVisible :: KRPCHS.UI.InputField -> RPCContext (Bool)
getInputFieldVisible thisArg = do
    let r = makeRequest "UI" "InputField_get_Visible" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getInputFieldVisibleStreamReq :: KRPCHS.UI.InputField -> RPCContext (KRPCStreamReq (Bool))
getInputFieldVisibleStreamReq thisArg = do
    let req = makeRequest "UI" "InputField_get_Visible" [makeArgument 0 thisArg]
    return (makeStream req)

getInputFieldVisibleStream :: KRPCHS.UI.InputField -> RPCContext (KRPCStream (Bool))
getInputFieldVisibleStream thisArg = requestStream =<< getInputFieldVisibleStreamReq thisArg 

{-
 - Whether the input field has been changed.This property is set to true when the user modifies the value of the input field.
 - A client script should reset the property to false in order to detect subsequent changes.
 -}
setInputFieldChanged :: KRPCHS.UI.InputField -> Bool -> RPCContext ()
setInputFieldChanged thisArg valueArg = do
    let r = makeRequest "UI" "InputField_set_Changed" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The value of the input field.
 -}
setInputFieldValue :: KRPCHS.UI.InputField -> Data.Text.Text -> RPCContext ()
setInputFieldValue thisArg valueArg = do
    let r = makeRequest "UI" "InputField_set_Value" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the UI object is visible.
 -}
setInputFieldVisible :: KRPCHS.UI.InputField -> Bool -> RPCContext ()
setInputFieldVisible thisArg valueArg = do
    let r = makeRequest "UI" "InputField_set_Visible" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Display a message on the screen.The message appears just like a stock message, for example quicksave or quickload messages.<param name="content">Message content.<param name="duration">Duration before the message disappears, in seconds.<param name="position">Position to display the message.
 -}
message :: Data.Text.Text -> Float -> KRPCHS.UI.MessagePosition -> RPCContext ()
message contentArg durationArg positionArg = do
    let r = makeRequest "UI" "Message" [makeArgument 0 contentArg, makeArgument 1 durationArg, makeArgument 2 positionArg]
    res <- sendRequest r
    processResponse res 

{-
 - Add a button to the panel.<param name="content">The label for the button.<param name="visible">Whether the button is visible.
 -}
panelAddButton :: KRPCHS.UI.Panel -> Data.Text.Text -> Bool -> RPCContext (KRPCHS.UI.Button)
panelAddButton thisArg contentArg visibleArg = do
    let r = makeRequest "UI" "Panel_AddButton" [makeArgument 0 thisArg, makeArgument 1 contentArg, makeArgument 2 visibleArg]
    res <- sendRequest r
    processResponse res

panelAddButtonStreamReq :: KRPCHS.UI.Panel -> Data.Text.Text -> Bool -> RPCContext (KRPCStreamReq (KRPCHS.UI.Button))
panelAddButtonStreamReq thisArg contentArg visibleArg = do
    let req = makeRequest "UI" "Panel_AddButton" [makeArgument 0 thisArg, makeArgument 1 contentArg, makeArgument 2 visibleArg]
    return (makeStream req)

panelAddButtonStream :: KRPCHS.UI.Panel -> Data.Text.Text -> Bool -> RPCContext (KRPCStream (KRPCHS.UI.Button))
panelAddButtonStream thisArg contentArg visibleArg = requestStream =<< panelAddButtonStreamReq thisArg contentArg visibleArg 

{-
 - Add an input field to the panel.<param name="visible">Whether the input field is visible.
 -}
panelAddInputField :: KRPCHS.UI.Panel -> Bool -> RPCContext (KRPCHS.UI.InputField)
panelAddInputField thisArg visibleArg = do
    let r = makeRequest "UI" "Panel_AddInputField" [makeArgument 0 thisArg, makeArgument 1 visibleArg]
    res <- sendRequest r
    processResponse res

panelAddInputFieldStreamReq :: KRPCHS.UI.Panel -> Bool -> RPCContext (KRPCStreamReq (KRPCHS.UI.InputField))
panelAddInputFieldStreamReq thisArg visibleArg = do
    let req = makeRequest "UI" "Panel_AddInputField" [makeArgument 0 thisArg, makeArgument 1 visibleArg]
    return (makeStream req)

panelAddInputFieldStream :: KRPCHS.UI.Panel -> Bool -> RPCContext (KRPCStream (KRPCHS.UI.InputField))
panelAddInputFieldStream thisArg visibleArg = requestStream =<< panelAddInputFieldStreamReq thisArg visibleArg 

{-
 - Create a panel within this panel.<param name="visible">Whether the new panel is visible.
 -}
panelAddPanel :: KRPCHS.UI.Panel -> Bool -> RPCContext (KRPCHS.UI.Panel)
panelAddPanel thisArg visibleArg = do
    let r = makeRequest "UI" "Panel_AddPanel" [makeArgument 0 thisArg, makeArgument 1 visibleArg]
    res <- sendRequest r
    processResponse res

panelAddPanelStreamReq :: KRPCHS.UI.Panel -> Bool -> RPCContext (KRPCStreamReq (KRPCHS.UI.Panel))
panelAddPanelStreamReq thisArg visibleArg = do
    let req = makeRequest "UI" "Panel_AddPanel" [makeArgument 0 thisArg, makeArgument 1 visibleArg]
    return (makeStream req)

panelAddPanelStream :: KRPCHS.UI.Panel -> Bool -> RPCContext (KRPCStream (KRPCHS.UI.Panel))
panelAddPanelStream thisArg visibleArg = requestStream =<< panelAddPanelStreamReq thisArg visibleArg 

{-
 - Add text to the panel.<param name="content">The text.<param name="visible">Whether the text is visible.
 -}
panelAddText :: KRPCHS.UI.Panel -> Data.Text.Text -> Bool -> RPCContext (KRPCHS.UI.Text)
panelAddText thisArg contentArg visibleArg = do
    let r = makeRequest "UI" "Panel_AddText" [makeArgument 0 thisArg, makeArgument 1 contentArg, makeArgument 2 visibleArg]
    res <- sendRequest r
    processResponse res

panelAddTextStreamReq :: KRPCHS.UI.Panel -> Data.Text.Text -> Bool -> RPCContext (KRPCStreamReq (KRPCHS.UI.Text))
panelAddTextStreamReq thisArg contentArg visibleArg = do
    let req = makeRequest "UI" "Panel_AddText" [makeArgument 0 thisArg, makeArgument 1 contentArg, makeArgument 2 visibleArg]
    return (makeStream req)

panelAddTextStream :: KRPCHS.UI.Panel -> Data.Text.Text -> Bool -> RPCContext (KRPCStream (KRPCHS.UI.Text))
panelAddTextStream thisArg contentArg visibleArg = requestStream =<< panelAddTextStreamReq thisArg contentArg visibleArg 

{-
 - Remove the UI object.
 -}
panelRemove :: KRPCHS.UI.Panel -> RPCContext ()
panelRemove thisArg = do
    let r = makeRequest "UI" "Panel_Remove" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - The rect transform for the panel.
 -}
getPanelRectTransform :: KRPCHS.UI.Panel -> RPCContext (KRPCHS.UI.RectTransform)
getPanelRectTransform thisArg = do
    let r = makeRequest "UI" "Panel_get_RectTransform" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPanelRectTransformStreamReq :: KRPCHS.UI.Panel -> RPCContext (KRPCStreamReq (KRPCHS.UI.RectTransform))
getPanelRectTransformStreamReq thisArg = do
    let req = makeRequest "UI" "Panel_get_RectTransform" [makeArgument 0 thisArg]
    return (makeStream req)

getPanelRectTransformStream :: KRPCHS.UI.Panel -> RPCContext (KRPCStream (KRPCHS.UI.RectTransform))
getPanelRectTransformStream thisArg = requestStream =<< getPanelRectTransformStreamReq thisArg 

{-
 - Whether the UI object is visible.
 -}
getPanelVisible :: KRPCHS.UI.Panel -> RPCContext (Bool)
getPanelVisible thisArg = do
    let r = makeRequest "UI" "Panel_get_Visible" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getPanelVisibleStreamReq :: KRPCHS.UI.Panel -> RPCContext (KRPCStreamReq (Bool))
getPanelVisibleStreamReq thisArg = do
    let req = makeRequest "UI" "Panel_get_Visible" [makeArgument 0 thisArg]
    return (makeStream req)

getPanelVisibleStream :: KRPCHS.UI.Panel -> RPCContext (KRPCStream (Bool))
getPanelVisibleStream thisArg = requestStream =<< getPanelVisibleStreamReq thisArg 

{-
 - Whether the UI object is visible.
 -}
setPanelVisible :: KRPCHS.UI.Panel -> Bool -> RPCContext ()
setPanelVisible thisArg valueArg = do
    let r = makeRequest "UI" "Panel_set_Visible" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The anchor point for the lower left corner of the rectangle defined as a fraction of the size of the parent rectangle.
 -}
getRectTransformAnchorMax :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double))
getRectTransformAnchorMax thisArg = do
    let r = makeRequest "UI" "RectTransform_get_AnchorMax" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRectTransformAnchorMaxStreamReq :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStreamReq ((Double, Double)))
getRectTransformAnchorMaxStreamReq thisArg = do
    let req = makeRequest "UI" "RectTransform_get_AnchorMax" [makeArgument 0 thisArg]
    return (makeStream req)

getRectTransformAnchorMaxStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double)))
getRectTransformAnchorMaxStream thisArg = requestStream =<< getRectTransformAnchorMaxStreamReq thisArg 

{-
 - The anchor point for the upper right corner of the rectangle defined as a fraction of the size of the parent rectangle.
 -}
getRectTransformAnchorMin :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double))
getRectTransformAnchorMin thisArg = do
    let r = makeRequest "UI" "RectTransform_get_AnchorMin" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRectTransformAnchorMinStreamReq :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStreamReq ((Double, Double)))
getRectTransformAnchorMinStreamReq thisArg = do
    let req = makeRequest "UI" "RectTransform_get_AnchorMin" [makeArgument 0 thisArg]
    return (makeStream req)

getRectTransformAnchorMinStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double)))
getRectTransformAnchorMinStream thisArg = requestStream =<< getRectTransformAnchorMinStreamReq thisArg 

{-
 - Position of the rectangles pivot point relative to the anchors.
 -}
getRectTransformLocalPosition :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double, Double))
getRectTransformLocalPosition thisArg = do
    let r = makeRequest "UI" "RectTransform_get_LocalPosition" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRectTransformLocalPositionStreamReq :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStreamReq ((Double, Double, Double)))
getRectTransformLocalPositionStreamReq thisArg = do
    let req = makeRequest "UI" "RectTransform_get_LocalPosition" [makeArgument 0 thisArg]
    return (makeStream req)

getRectTransformLocalPositionStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double, Double)))
getRectTransformLocalPositionStream thisArg = requestStream =<< getRectTransformLocalPositionStreamReq thisArg 

{-
 - Position of the rectangles lower left corner relative to the anchors.
 -}
getRectTransformLowerLeft :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double))
getRectTransformLowerLeft thisArg = do
    let r = makeRequest "UI" "RectTransform_get_LowerLeft" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRectTransformLowerLeftStreamReq :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStreamReq ((Double, Double)))
getRectTransformLowerLeftStreamReq thisArg = do
    let req = makeRequest "UI" "RectTransform_get_LowerLeft" [makeArgument 0 thisArg]
    return (makeStream req)

getRectTransformLowerLeftStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double)))
getRectTransformLowerLeftStream thisArg = requestStream =<< getRectTransformLowerLeftStreamReq thisArg 

{-
 - Location of the pivot point around which the rectangle rotates, defined as a fraction of the size of the rectangle itself.
 -}
getRectTransformPivot :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double))
getRectTransformPivot thisArg = do
    let r = makeRequest "UI" "RectTransform_get_Pivot" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRectTransformPivotStreamReq :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStreamReq ((Double, Double)))
getRectTransformPivotStreamReq thisArg = do
    let req = makeRequest "UI" "RectTransform_get_Pivot" [makeArgument 0 thisArg]
    return (makeStream req)

getRectTransformPivotStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double)))
getRectTransformPivotStream thisArg = requestStream =<< getRectTransformPivotStreamReq thisArg 

{-
 - Position of the rectangles pivot point relative to the anchors.
 -}
getRectTransformPosition :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double))
getRectTransformPosition thisArg = do
    let r = makeRequest "UI" "RectTransform_get_Position" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRectTransformPositionStreamReq :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStreamReq ((Double, Double)))
getRectTransformPositionStreamReq thisArg = do
    let req = makeRequest "UI" "RectTransform_get_Position" [makeArgument 0 thisArg]
    return (makeStream req)

getRectTransformPositionStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double)))
getRectTransformPositionStream thisArg = requestStream =<< getRectTransformPositionStreamReq thisArg 

{-
 - Rotation, as a quaternion, of the object around its pivot point.
 -}
getRectTransformRotation :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double, Double, Double))
getRectTransformRotation thisArg = do
    let r = makeRequest "UI" "RectTransform_get_Rotation" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRectTransformRotationStreamReq :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStreamReq ((Double, Double, Double, Double)))
getRectTransformRotationStreamReq thisArg = do
    let req = makeRequest "UI" "RectTransform_get_Rotation" [makeArgument 0 thisArg]
    return (makeStream req)

getRectTransformRotationStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
getRectTransformRotationStream thisArg = requestStream =<< getRectTransformRotationStreamReq thisArg 

{-
 - Scale factor applied to the object in the x, y and z dimensions.
 -}
getRectTransformScale :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double, Double))
getRectTransformScale thisArg = do
    let r = makeRequest "UI" "RectTransform_get_Scale" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRectTransformScaleStreamReq :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStreamReq ((Double, Double, Double)))
getRectTransformScaleStreamReq thisArg = do
    let req = makeRequest "UI" "RectTransform_get_Scale" [makeArgument 0 thisArg]
    return (makeStream req)

getRectTransformScaleStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double, Double)))
getRectTransformScaleStream thisArg = requestStream =<< getRectTransformScaleStreamReq thisArg 

{-
 - Width and height of the rectangle.
 -}
getRectTransformSize :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double))
getRectTransformSize thisArg = do
    let r = makeRequest "UI" "RectTransform_get_Size" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRectTransformSizeStreamReq :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStreamReq ((Double, Double)))
getRectTransformSizeStreamReq thisArg = do
    let req = makeRequest "UI" "RectTransform_get_Size" [makeArgument 0 thisArg]
    return (makeStream req)

getRectTransformSizeStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double)))
getRectTransformSizeStream thisArg = requestStream =<< getRectTransformSizeStreamReq thisArg 

{-
 - Position of the rectangles upper right corner relative to the anchors.
 -}
getRectTransformUpperRight :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double))
getRectTransformUpperRight thisArg = do
    let r = makeRequest "UI" "RectTransform_get_UpperRight" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getRectTransformUpperRightStreamReq :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStreamReq ((Double, Double)))
getRectTransformUpperRightStreamReq thisArg = do
    let req = makeRequest "UI" "RectTransform_get_UpperRight" [makeArgument 0 thisArg]
    return (makeStream req)

getRectTransformUpperRightStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double)))
getRectTransformUpperRightStream thisArg = requestStream =<< getRectTransformUpperRightStreamReq thisArg 

{-
 - Set the minimum and maximum anchor points as a fraction of the size of the parent rectangle.
 -}
setRectTransformAnchor :: KRPCHS.UI.RectTransform -> (Double, Double) -> RPCContext ()
setRectTransformAnchor thisArg valueArg = do
    let r = makeRequest "UI" "RectTransform_set_Anchor" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The anchor point for the lower left corner of the rectangle defined as a fraction of the size of the parent rectangle.
 -}
setRectTransformAnchorMax :: KRPCHS.UI.RectTransform -> (Double, Double) -> RPCContext ()
setRectTransformAnchorMax thisArg valueArg = do
    let r = makeRequest "UI" "RectTransform_set_AnchorMax" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The anchor point for the upper right corner of the rectangle defined as a fraction of the size of the parent rectangle.
 -}
setRectTransformAnchorMin :: KRPCHS.UI.RectTransform -> (Double, Double) -> RPCContext ()
setRectTransformAnchorMin thisArg valueArg = do
    let r = makeRequest "UI" "RectTransform_set_AnchorMin" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Position of the rectangles pivot point relative to the anchors.
 -}
setRectTransformLocalPosition :: KRPCHS.UI.RectTransform -> (Double, Double, Double) -> RPCContext ()
setRectTransformLocalPosition thisArg valueArg = do
    let r = makeRequest "UI" "RectTransform_set_LocalPosition" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Position of the rectangles lower left corner relative to the anchors.
 -}
setRectTransformLowerLeft :: KRPCHS.UI.RectTransform -> (Double, Double) -> RPCContext ()
setRectTransformLowerLeft thisArg valueArg = do
    let r = makeRequest "UI" "RectTransform_set_LowerLeft" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Location of the pivot point around which the rectangle rotates, defined as a fraction of the size of the rectangle itself.
 -}
setRectTransformPivot :: KRPCHS.UI.RectTransform -> (Double, Double) -> RPCContext ()
setRectTransformPivot thisArg valueArg = do
    let r = makeRequest "UI" "RectTransform_set_Pivot" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Position of the rectangles pivot point relative to the anchors.
 -}
setRectTransformPosition :: KRPCHS.UI.RectTransform -> (Double, Double) -> RPCContext ()
setRectTransformPosition thisArg valueArg = do
    let r = makeRequest "UI" "RectTransform_set_Position" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Rotation, as a quaternion, of the object around its pivot point.
 -}
setRectTransformRotation :: KRPCHS.UI.RectTransform -> (Double, Double, Double, Double) -> RPCContext ()
setRectTransformRotation thisArg valueArg = do
    let r = makeRequest "UI" "RectTransform_set_Rotation" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Scale factor applied to the object in the x, y and z dimensions.
 -}
setRectTransformScale :: KRPCHS.UI.RectTransform -> (Double, Double, Double) -> RPCContext ()
setRectTransformScale thisArg valueArg = do
    let r = makeRequest "UI" "RectTransform_set_Scale" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Width and height of the rectangle.
 -}
setRectTransformSize :: KRPCHS.UI.RectTransform -> (Double, Double) -> RPCContext ()
setRectTransformSize thisArg valueArg = do
    let r = makeRequest "UI" "RectTransform_set_Size" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Position of the rectangles upper right corner relative to the anchors.
 -}
setRectTransformUpperRight :: KRPCHS.UI.RectTransform -> (Double, Double) -> RPCContext ()
setRectTransformUpperRight thisArg valueArg = do
    let r = makeRequest "UI" "RectTransform_set_UpperRight" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Remove the UI object.
 -}
textRemove :: KRPCHS.UI.Text -> RPCContext ()
textRemove thisArg = do
    let r = makeRequest "UI" "Text_Remove" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res 

{-
 - Alignment.
 -}
getTextAlignment :: KRPCHS.UI.Text -> RPCContext (KRPCHS.UI.TextAnchor)
getTextAlignment thisArg = do
    let r = makeRequest "UI" "Text_get_Alignment" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextAlignmentStreamReq :: KRPCHS.UI.Text -> RPCContext (KRPCStreamReq (KRPCHS.UI.TextAnchor))
getTextAlignmentStreamReq thisArg = do
    let req = makeRequest "UI" "Text_get_Alignment" [makeArgument 0 thisArg]
    return (makeStream req)

getTextAlignmentStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream (KRPCHS.UI.TextAnchor))
getTextAlignmentStream thisArg = requestStream =<< getTextAlignmentStreamReq thisArg 

{-
 - A list of all available fonts.
 -}
getTextAvailableFonts :: KRPCHS.UI.Text -> RPCContext ([Data.Text.Text])
getTextAvailableFonts thisArg = do
    let r = makeRequest "UI" "Text_get_AvailableFonts" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextAvailableFontsStreamReq :: KRPCHS.UI.Text -> RPCContext (KRPCStreamReq ([Data.Text.Text]))
getTextAvailableFontsStreamReq thisArg = do
    let req = makeRequest "UI" "Text_get_AvailableFonts" [makeArgument 0 thisArg]
    return (makeStream req)

getTextAvailableFontsStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream ([Data.Text.Text]))
getTextAvailableFontsStream thisArg = requestStream =<< getTextAvailableFontsStreamReq thisArg 

{-
 - Set the color
 -}
getTextColor :: KRPCHS.UI.Text -> RPCContext ((Double, Double, Double))
getTextColor thisArg = do
    let r = makeRequest "UI" "Text_get_Color" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextColorStreamReq :: KRPCHS.UI.Text -> RPCContext (KRPCStreamReq ((Double, Double, Double)))
getTextColorStreamReq thisArg = do
    let req = makeRequest "UI" "Text_get_Color" [makeArgument 0 thisArg]
    return (makeStream req)

getTextColorStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream ((Double, Double, Double)))
getTextColorStream thisArg = requestStream =<< getTextColorStreamReq thisArg 

{-
 - The text string
 -}
getTextContent :: KRPCHS.UI.Text -> RPCContext (Data.Text.Text)
getTextContent thisArg = do
    let r = makeRequest "UI" "Text_get_Content" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextContentStreamReq :: KRPCHS.UI.Text -> RPCContext (KRPCStreamReq (Data.Text.Text))
getTextContentStreamReq thisArg = do
    let req = makeRequest "UI" "Text_get_Content" [makeArgument 0 thisArg]
    return (makeStream req)

getTextContentStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream (Data.Text.Text))
getTextContentStream thisArg = requestStream =<< getTextContentStreamReq thisArg 

{-
 - Name of the font
 -}
getTextFont :: KRPCHS.UI.Text -> RPCContext (Data.Text.Text)
getTextFont thisArg = do
    let r = makeRequest "UI" "Text_get_Font" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextFontStreamReq :: KRPCHS.UI.Text -> RPCContext (KRPCStreamReq (Data.Text.Text))
getTextFontStreamReq thisArg = do
    let req = makeRequest "UI" "Text_get_Font" [makeArgument 0 thisArg]
    return (makeStream req)

getTextFontStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream (Data.Text.Text))
getTextFontStream thisArg = requestStream =<< getTextFontStreamReq thisArg 

{-
 - Line spacing.
 -}
getTextLineSpacing :: KRPCHS.UI.Text -> RPCContext (Float)
getTextLineSpacing thisArg = do
    let r = makeRequest "UI" "Text_get_LineSpacing" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextLineSpacingStreamReq :: KRPCHS.UI.Text -> RPCContext (KRPCStreamReq (Float))
getTextLineSpacingStreamReq thisArg = do
    let req = makeRequest "UI" "Text_get_LineSpacing" [makeArgument 0 thisArg]
    return (makeStream req)

getTextLineSpacingStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream (Float))
getTextLineSpacingStream thisArg = requestStream =<< getTextLineSpacingStreamReq thisArg 

{-
 - The rect transform for the text.
 -}
getTextRectTransform :: KRPCHS.UI.Text -> RPCContext (KRPCHS.UI.RectTransform)
getTextRectTransform thisArg = do
    let r = makeRequest "UI" "Text_get_RectTransform" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextRectTransformStreamReq :: KRPCHS.UI.Text -> RPCContext (KRPCStreamReq (KRPCHS.UI.RectTransform))
getTextRectTransformStreamReq thisArg = do
    let req = makeRequest "UI" "Text_get_RectTransform" [makeArgument 0 thisArg]
    return (makeStream req)

getTextRectTransformStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream (KRPCHS.UI.RectTransform))
getTextRectTransformStream thisArg = requestStream =<< getTextRectTransformStreamReq thisArg 

{-
 - Font size.
 -}
getTextSize :: KRPCHS.UI.Text -> RPCContext (Data.Int.Int32)
getTextSize thisArg = do
    let r = makeRequest "UI" "Text_get_Size" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextSizeStreamReq :: KRPCHS.UI.Text -> RPCContext (KRPCStreamReq (Data.Int.Int32))
getTextSizeStreamReq thisArg = do
    let req = makeRequest "UI" "Text_get_Size" [makeArgument 0 thisArg]
    return (makeStream req)

getTextSizeStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream (Data.Int.Int32))
getTextSizeStream thisArg = requestStream =<< getTextSizeStreamReq thisArg 

{-
 - Font style.
 -}
getTextStyle :: KRPCHS.UI.Text -> RPCContext (KRPCHS.UI.FontStyle)
getTextStyle thisArg = do
    let r = makeRequest "UI" "Text_get_Style" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextStyleStreamReq :: KRPCHS.UI.Text -> RPCContext (KRPCStreamReq (KRPCHS.UI.FontStyle))
getTextStyleStreamReq thisArg = do
    let req = makeRequest "UI" "Text_get_Style" [makeArgument 0 thisArg]
    return (makeStream req)

getTextStyleStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream (KRPCHS.UI.FontStyle))
getTextStyleStream thisArg = requestStream =<< getTextStyleStreamReq thisArg 

{-
 - Whether the UI object is visible.
 -}
getTextVisible :: KRPCHS.UI.Text -> RPCContext (Bool)
getTextVisible thisArg = do
    let r = makeRequest "UI" "Text_get_Visible" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse res

getTextVisibleStreamReq :: KRPCHS.UI.Text -> RPCContext (KRPCStreamReq (Bool))
getTextVisibleStreamReq thisArg = do
    let req = makeRequest "UI" "Text_get_Visible" [makeArgument 0 thisArg]
    return (makeStream req)

getTextVisibleStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream (Bool))
getTextVisibleStream thisArg = requestStream =<< getTextVisibleStreamReq thisArg 

{-
 - Alignment.
 -}
setTextAlignment :: KRPCHS.UI.Text -> KRPCHS.UI.TextAnchor -> RPCContext ()
setTextAlignment thisArg valueArg = do
    let r = makeRequest "UI" "Text_set_Alignment" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Set the color
 -}
setTextColor :: KRPCHS.UI.Text -> (Double, Double, Double) -> RPCContext ()
setTextColor thisArg valueArg = do
    let r = makeRequest "UI" "Text_set_Color" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The text string
 -}
setTextContent :: KRPCHS.UI.Text -> Data.Text.Text -> RPCContext ()
setTextContent thisArg valueArg = do
    let r = makeRequest "UI" "Text_set_Content" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Name of the font
 -}
setTextFont :: KRPCHS.UI.Text -> Data.Text.Text -> RPCContext ()
setTextFont thisArg valueArg = do
    let r = makeRequest "UI" "Text_set_Font" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Line spacing.
 -}
setTextLineSpacing :: KRPCHS.UI.Text -> Float -> RPCContext ()
setTextLineSpacing thisArg valueArg = do
    let r = makeRequest "UI" "Text_set_LineSpacing" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Font size.
 -}
setTextSize :: KRPCHS.UI.Text -> Data.Int.Int32 -> RPCContext ()
setTextSize thisArg valueArg = do
    let r = makeRequest "UI" "Text_set_Size" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Font style.
 -}
setTextStyle :: KRPCHS.UI.Text -> KRPCHS.UI.FontStyle -> RPCContext ()
setTextStyle thisArg valueArg = do
    let r = makeRequest "UI" "Text_set_Style" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - Whether the UI object is visible.
 -}
setTextVisible :: KRPCHS.UI.Text -> Bool -> RPCContext ()
setTextVisible thisArg valueArg = do
    let r = makeRequest "UI" "Text_set_Visible" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse res 

{-
 - The stock UI canvas.
 -}
getStockCanvas :: RPCContext (KRPCHS.UI.Canvas)
getStockCanvas  = do
    let r = makeRequest "UI" "get_StockCanvas" []
    res <- sendRequest r
    processResponse res

getStockCanvasStreamReq :: RPCContext (KRPCStreamReq (KRPCHS.UI.Canvas))
getStockCanvasStreamReq  = do
    let req = makeRequest "UI" "get_StockCanvas" []
    return (makeStream req)

getStockCanvasStream :: RPCContext (KRPCStream (KRPCHS.UI.Canvas))
getStockCanvasStream  = requestStream =<< getStockCanvasStreamReq  

