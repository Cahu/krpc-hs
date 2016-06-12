module KRPCHS.UI
( FontStyle(..)
, MessagePosition(..)
, TextAlignment(..)
, TextAnchor(..)
, Button
, InputField
, Panel
, RectTransform
, Text
, addPanel
, addPanelStream
, buttonRemove
, getButtonClicked
, getButtonClickedStream
, getButtonRectTransform
, getButtonRectTransformStream
, getButtonText
, getButtonTextStream
, getButtonVisible
, getButtonVisibleStream
, setButtonClicked
, setButtonVisible
, clear
, inputFieldRemove
, getInputFieldChanged
, getInputFieldChangedStream
, getInputFieldRectTransform
, getInputFieldRectTransformStream
, getInputFieldText
, getInputFieldTextStream
, getInputFieldValue
, getInputFieldValueStream
, getInputFieldVisible
, getInputFieldVisibleStream
, setInputFieldChanged
, setInputFieldValue
, setInputFieldVisible
, message
, panelAddButton
, panelAddButtonStream
, panelAddInputField
, panelAddInputFieldStream
, panelAddPanel
, panelAddPanelStream
, panelAddText
, panelAddTextStream
, panelRemove
, getPanelRectTransform
, getPanelRectTransformStream
, getPanelVisible
, getPanelVisibleStream
, setPanelVisible
, getRectTransformAnchorMax
, getRectTransformAnchorMaxStream
, getRectTransformAnchorMin
, getRectTransformAnchorMinStream
, getRectTransformLocalPosition
, getRectTransformLocalPositionStream
, getRectTransformLowerLeft
, getRectTransformLowerLeftStream
, getRectTransformPivot
, getRectTransformPivotStream
, getRectTransformPosition
, getRectTransformPositionStream
, getRectTransformRotation
, getRectTransformRotationStream
, getRectTransformScale
, getRectTransformScaleStream
, getRectTransformSize
, getRectTransformSizeStream
, getRectTransformUpperRight
, getRectTransformUpperRightStream
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
, getTextAvailableFonts
, getTextAvailableFontsStream
, getTextColor
, getTextColorStream
, getTextContent
, getTextContentStream
, getTextFont
, getTextFontStream
, getTextLineSpacing
, getTextLineSpacingStream
, getTextRectTransform
, getTextRectTransformStream
, getTextSize
, getTextSizeStream
, getTextStyle
, getTextStyleStream
, getTextVisible
, getTextVisibleStream
, setTextAlignment
, setTextColor
, setTextContent
, setTextFont
, setTextLineSpacing
, setTextSize
, setTextStyle
, setTextVisible
, getRectTransform
, getRectTransformStream
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
 - An input field. See <see cref="M:UI.Panel.AddInputField" />.
 -}
newtype InputField = InputField { inputFieldId :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable InputField where
    encodePb   = encodePb . inputFieldId
    decodePb b = InputField <$> decodePb b

instance KRPCResponseExtractable InputField

{-
 - A container for user interface elements. See <see cref="M:UI.AddPanel" />.
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
 - Create a new container for user interface elements.<param name="visible">Whether the panel is visible.
 -}
addPanel :: Bool -> RPCContext (KRPCHS.UI.Panel)
addPanel visibleArg = do
    let r = makeRequest "UI" "AddPanel" [makeArgument 0 visibleArg]
    res <- sendRequest r
    processResponse extract res 

addPanelStream :: Bool -> RPCContext (KRPCStream (KRPCHS.UI.Panel))
addPanelStream visibleArg = do
    let r = makeRequest "UI" "AddPanel" [makeArgument 0 visibleArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Remove the UI object.
 -}
buttonRemove :: KRPCHS.UI.Button -> RPCContext (Bool)
buttonRemove thisArg = do
    let r = makeRequest "UI" "Button_Remove" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the button has been clicked.This property is set to true when the user clicks the button.
 - A client script should reset the property to false in order to detect subsequent button presses.
 -}
getButtonClicked :: KRPCHS.UI.Button -> RPCContext (Bool)
getButtonClicked thisArg = do
    let r = makeRequest "UI" "Button_get_Clicked" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getButtonClickedStream :: KRPCHS.UI.Button -> RPCContext (KRPCStream (Bool))
getButtonClickedStream thisArg = do
    let r = makeRequest "UI" "Button_get_Clicked" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The rect transform for the text.
 -}
getButtonRectTransform :: KRPCHS.UI.Button -> RPCContext (KRPCHS.UI.RectTransform)
getButtonRectTransform thisArg = do
    let r = makeRequest "UI" "Button_get_RectTransform" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getButtonRectTransformStream :: KRPCHS.UI.Button -> RPCContext (KRPCStream (KRPCHS.UI.RectTransform))
getButtonRectTransformStream thisArg = do
    let r = makeRequest "UI" "Button_get_RectTransform" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The text for the button.
 -}
getButtonText :: KRPCHS.UI.Button -> RPCContext (KRPCHS.UI.Text)
getButtonText thisArg = do
    let r = makeRequest "UI" "Button_get_Text" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getButtonTextStream :: KRPCHS.UI.Button -> RPCContext (KRPCStream (KRPCHS.UI.Text))
getButtonTextStream thisArg = do
    let r = makeRequest "UI" "Button_get_Text" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the UI object is visible.
 -}
getButtonVisible :: KRPCHS.UI.Button -> RPCContext (Bool)
getButtonVisible thisArg = do
    let r = makeRequest "UI" "Button_get_Visible" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getButtonVisibleStream :: KRPCHS.UI.Button -> RPCContext (KRPCStream (Bool))
getButtonVisibleStream thisArg = do
    let r = makeRequest "UI" "Button_get_Visible" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the button has been clicked.This property is set to true when the user clicks the button.
 - A client script should reset the property to false in order to detect subsequent button presses.
 -}
setButtonClicked :: KRPCHS.UI.Button -> Bool -> RPCContext (Bool)
setButtonClicked thisArg valueArg = do
    let r = makeRequest "UI" "Button_set_Clicked" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the UI object is visible.
 -}
setButtonVisible :: KRPCHS.UI.Button -> Bool -> RPCContext (Bool)
setButtonVisible thisArg valueArg = do
    let r = makeRequest "UI" "Button_set_Visible" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Remove all user interface elements.<param name="clientOnly">If true, only remove objects created by the calling client.
 -}
clear :: Bool -> RPCContext (Bool)
clear clientOnlyArg = do
    let r = makeRequest "UI" "Clear" [makeArgument 0 clientOnlyArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Remove the UI object.
 -}
inputFieldRemove :: KRPCHS.UI.InputField -> RPCContext (Bool)
inputFieldRemove thisArg = do
    let r = makeRequest "UI" "InputField_Remove" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the input field has been changed.This property is set to true when the user modifies the value of the input field.
 - A client script should reset the property to false in order to detect subsequent changes.
 -}
getInputFieldChanged :: KRPCHS.UI.InputField -> RPCContext (Bool)
getInputFieldChanged thisArg = do
    let r = makeRequest "UI" "InputField_get_Changed" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getInputFieldChangedStream :: KRPCHS.UI.InputField -> RPCContext (KRPCStream (Bool))
getInputFieldChangedStream thisArg = do
    let r = makeRequest "UI" "InputField_get_Changed" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The rect transform for the input field.
 -}
getInputFieldRectTransform :: KRPCHS.UI.InputField -> RPCContext (KRPCHS.UI.RectTransform)
getInputFieldRectTransform thisArg = do
    let r = makeRequest "UI" "InputField_get_RectTransform" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getInputFieldRectTransformStream :: KRPCHS.UI.InputField -> RPCContext (KRPCStream (KRPCHS.UI.RectTransform))
getInputFieldRectTransformStream thisArg = do
    let r = makeRequest "UI" "InputField_get_RectTransform" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The text component of the input field.Use <see cref="M:UI.InputField.Value" /> to get and set the value in the field.
 - This object can be used to alter the style of the input field's text.
 -}
getInputFieldText :: KRPCHS.UI.InputField -> RPCContext (KRPCHS.UI.Text)
getInputFieldText thisArg = do
    let r = makeRequest "UI" "InputField_get_Text" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getInputFieldTextStream :: KRPCHS.UI.InputField -> RPCContext (KRPCStream (KRPCHS.UI.Text))
getInputFieldTextStream thisArg = do
    let r = makeRequest "UI" "InputField_get_Text" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The value of the input field.
 -}
getInputFieldValue :: KRPCHS.UI.InputField -> RPCContext (Data.Text.Text)
getInputFieldValue thisArg = do
    let r = makeRequest "UI" "InputField_get_Value" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getInputFieldValueStream :: KRPCHS.UI.InputField -> RPCContext (KRPCStream (Data.Text.Text))
getInputFieldValueStream thisArg = do
    let r = makeRequest "UI" "InputField_get_Value" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the UI object is visible.
 -}
getInputFieldVisible :: KRPCHS.UI.InputField -> RPCContext (Bool)
getInputFieldVisible thisArg = do
    let r = makeRequest "UI" "InputField_get_Visible" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getInputFieldVisibleStream :: KRPCHS.UI.InputField -> RPCContext (KRPCStream (Bool))
getInputFieldVisibleStream thisArg = do
    let r = makeRequest "UI" "InputField_get_Visible" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the input field has been changed.This property is set to true when the user modifies the value of the input field.
 - A client script should reset the property to false in order to detect subsequent changes.
 -}
setInputFieldChanged :: KRPCHS.UI.InputField -> Bool -> RPCContext (Bool)
setInputFieldChanged thisArg valueArg = do
    let r = makeRequest "UI" "InputField_set_Changed" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The value of the input field.
 -}
setInputFieldValue :: KRPCHS.UI.InputField -> Data.Text.Text -> RPCContext (Bool)
setInputFieldValue thisArg valueArg = do
    let r = makeRequest "UI" "InputField_set_Value" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the UI object is visible.
 -}
setInputFieldVisible :: KRPCHS.UI.InputField -> Bool -> RPCContext (Bool)
setInputFieldVisible thisArg valueArg = do
    let r = makeRequest "UI" "InputField_set_Visible" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Display a message on the screen.The message appears just like a stock message, for example quicksave or quickload messages.<param name="content">Message content.<param name="duration">Duration before the message disappears, in seconds.<param name="position">Position to display the message.
 -}
message :: Data.Text.Text -> Float -> KRPCHS.UI.MessagePosition -> RPCContext (Bool)
message contentArg durationArg positionArg = do
    let r = makeRequest "UI" "Message" [makeArgument 0 contentArg, makeArgument 1 durationArg, makeArgument 2 positionArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Add a button to the panel.<param name="content">The label for the button.<param name="visible">Whether the button is visible.
 -}
panelAddButton :: KRPCHS.UI.Panel -> Data.Text.Text -> Bool -> RPCContext (KRPCHS.UI.Button)
panelAddButton thisArg contentArg visibleArg = do
    let r = makeRequest "UI" "Panel_AddButton" [makeArgument 0 thisArg, makeArgument 1 contentArg, makeArgument 2 visibleArg]
    res <- sendRequest r
    processResponse extract res 

panelAddButtonStream :: KRPCHS.UI.Panel -> Data.Text.Text -> Bool -> RPCContext (KRPCStream (KRPCHS.UI.Button))
panelAddButtonStream thisArg contentArg visibleArg = do
    let r = makeRequest "UI" "Panel_AddButton" [makeArgument 0 thisArg, makeArgument 1 contentArg, makeArgument 2 visibleArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Add an input field to the panel.<param name="visible">Whether the input field is visible.
 -}
panelAddInputField :: KRPCHS.UI.Panel -> Bool -> RPCContext (KRPCHS.UI.InputField)
panelAddInputField thisArg visibleArg = do
    let r = makeRequest "UI" "Panel_AddInputField" [makeArgument 0 thisArg, makeArgument 1 visibleArg]
    res <- sendRequest r
    processResponse extract res 

panelAddInputFieldStream :: KRPCHS.UI.Panel -> Bool -> RPCContext (KRPCStream (KRPCHS.UI.InputField))
panelAddInputFieldStream thisArg visibleArg = do
    let r = makeRequest "UI" "Panel_AddInputField" [makeArgument 0 thisArg, makeArgument 1 visibleArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Create a panel within this panel.<param name="visible">Whether the new panel is visible.
 -}
panelAddPanel :: KRPCHS.UI.Panel -> Bool -> RPCContext (KRPCHS.UI.Panel)
panelAddPanel thisArg visibleArg = do
    let r = makeRequest "UI" "Panel_AddPanel" [makeArgument 0 thisArg, makeArgument 1 visibleArg]
    res <- sendRequest r
    processResponse extract res 

panelAddPanelStream :: KRPCHS.UI.Panel -> Bool -> RPCContext (KRPCStream (KRPCHS.UI.Panel))
panelAddPanelStream thisArg visibleArg = do
    let r = makeRequest "UI" "Panel_AddPanel" [makeArgument 0 thisArg, makeArgument 1 visibleArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Add text to the panel.<param name="content">The text.<param name="visible">Whether the text is visible.
 -}
panelAddText :: KRPCHS.UI.Panel -> Data.Text.Text -> Bool -> RPCContext (KRPCHS.UI.Text)
panelAddText thisArg contentArg visibleArg = do
    let r = makeRequest "UI" "Panel_AddText" [makeArgument 0 thisArg, makeArgument 1 contentArg, makeArgument 2 visibleArg]
    res <- sendRequest r
    processResponse extract res 

panelAddTextStream :: KRPCHS.UI.Panel -> Data.Text.Text -> Bool -> RPCContext (KRPCStream (KRPCHS.UI.Text))
panelAddTextStream thisArg contentArg visibleArg = do
    let r = makeRequest "UI" "Panel_AddText" [makeArgument 0 thisArg, makeArgument 1 contentArg, makeArgument 2 visibleArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Remove the UI object.
 -}
panelRemove :: KRPCHS.UI.Panel -> RPCContext (Bool)
panelRemove thisArg = do
    let r = makeRequest "UI" "Panel_Remove" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The rect transform for the panel.
 -}
getPanelRectTransform :: KRPCHS.UI.Panel -> RPCContext (KRPCHS.UI.RectTransform)
getPanelRectTransform thisArg = do
    let r = makeRequest "UI" "Panel_get_RectTransform" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPanelRectTransformStream :: KRPCHS.UI.Panel -> RPCContext (KRPCStream (KRPCHS.UI.RectTransform))
getPanelRectTransformStream thisArg = do
    let r = makeRequest "UI" "Panel_get_RectTransform" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the UI object is visible.
 -}
getPanelVisible :: KRPCHS.UI.Panel -> RPCContext (Bool)
getPanelVisible thisArg = do
    let r = makeRequest "UI" "Panel_get_Visible" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getPanelVisibleStream :: KRPCHS.UI.Panel -> RPCContext (KRPCStream (Bool))
getPanelVisibleStream thisArg = do
    let r = makeRequest "UI" "Panel_get_Visible" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the UI object is visible.
 -}
setPanelVisible :: KRPCHS.UI.Panel -> Bool -> RPCContext (Bool)
setPanelVisible thisArg valueArg = do
    let r = makeRequest "UI" "Panel_set_Visible" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The anchor point for the lower left corner of the rectangle defined as a fraction of the size of the parent rectangle.
 -}
getRectTransformAnchorMax :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double))
getRectTransformAnchorMax thisArg = do
    let r = makeRequest "UI" "RectTransform_get_AnchorMax" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRectTransformAnchorMaxStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double)))
getRectTransformAnchorMaxStream thisArg = do
    let r = makeRequest "UI" "RectTransform_get_AnchorMax" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The anchor point for the upper right corner of the rectangle defined as a fraction of the size of the parent rectangle.
 -}
getRectTransformAnchorMin :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double))
getRectTransformAnchorMin thisArg = do
    let r = makeRequest "UI" "RectTransform_get_AnchorMin" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRectTransformAnchorMinStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double)))
getRectTransformAnchorMinStream thisArg = do
    let r = makeRequest "UI" "RectTransform_get_AnchorMin" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Position of the rectangles pivot point relative to the anchors.
 -}
getRectTransformLocalPosition :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double, Double))
getRectTransformLocalPosition thisArg = do
    let r = makeRequest "UI" "RectTransform_get_LocalPosition" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRectTransformLocalPositionStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double, Double)))
getRectTransformLocalPositionStream thisArg = do
    let r = makeRequest "UI" "RectTransform_get_LocalPosition" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Position of the rectangles lower left corner relative to the anchors.
 -}
getRectTransformLowerLeft :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double))
getRectTransformLowerLeft thisArg = do
    let r = makeRequest "UI" "RectTransform_get_LowerLeft" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRectTransformLowerLeftStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double)))
getRectTransformLowerLeftStream thisArg = do
    let r = makeRequest "UI" "RectTransform_get_LowerLeft" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Location of the pivot point around which the rectangle rotates, defined as a fraction of the size of the rectangle itself.
 -}
getRectTransformPivot :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double))
getRectTransformPivot thisArg = do
    let r = makeRequest "UI" "RectTransform_get_Pivot" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRectTransformPivotStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double)))
getRectTransformPivotStream thisArg = do
    let r = makeRequest "UI" "RectTransform_get_Pivot" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Position of the rectangles pivot point relative to the anchors.
 -}
getRectTransformPosition :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double))
getRectTransformPosition thisArg = do
    let r = makeRequest "UI" "RectTransform_get_Position" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRectTransformPositionStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double)))
getRectTransformPositionStream thisArg = do
    let r = makeRequest "UI" "RectTransform_get_Position" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Rotation, as a quaternion, of the object around its pivot point.
 -}
getRectTransformRotation :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double, Double, Double))
getRectTransformRotation thisArg = do
    let r = makeRequest "UI" "RectTransform_get_Rotation" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRectTransformRotationStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double, Double, Double)))
getRectTransformRotationStream thisArg = do
    let r = makeRequest "UI" "RectTransform_get_Rotation" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Scale factor applied to the object in the x, y and z dimensions.
 -}
getRectTransformScale :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double, Double))
getRectTransformScale thisArg = do
    let r = makeRequest "UI" "RectTransform_get_Scale" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRectTransformScaleStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double, Double)))
getRectTransformScaleStream thisArg = do
    let r = makeRequest "UI" "RectTransform_get_Scale" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Width and height of the rectangle.
 -}
getRectTransformSize :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double))
getRectTransformSize thisArg = do
    let r = makeRequest "UI" "RectTransform_get_Size" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRectTransformSizeStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double)))
getRectTransformSizeStream thisArg = do
    let r = makeRequest "UI" "RectTransform_get_Size" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Position of the rectangles upper right corner relative to the anchors.
 -}
getRectTransformUpperRight :: KRPCHS.UI.RectTransform -> RPCContext ((Double, Double))
getRectTransformUpperRight thisArg = do
    let r = makeRequest "UI" "RectTransform_get_UpperRight" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getRectTransformUpperRightStream :: KRPCHS.UI.RectTransform -> RPCContext (KRPCStream ((Double, Double)))
getRectTransformUpperRightStream thisArg = do
    let r = makeRequest "UI" "RectTransform_get_UpperRight" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Set the minimum and maximum anchor points as a fraction of the size of the parent rectangle.
 -}
setRectTransformAnchor :: KRPCHS.UI.RectTransform -> (Double, Double) -> RPCContext (Bool)
setRectTransformAnchor thisArg valueArg = do
    let r = makeRequest "UI" "RectTransform_set_Anchor" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The anchor point for the lower left corner of the rectangle defined as a fraction of the size of the parent rectangle.
 -}
setRectTransformAnchorMax :: KRPCHS.UI.RectTransform -> (Double, Double) -> RPCContext (Bool)
setRectTransformAnchorMax thisArg valueArg = do
    let r = makeRequest "UI" "RectTransform_set_AnchorMax" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The anchor point for the upper right corner of the rectangle defined as a fraction of the size of the parent rectangle.
 -}
setRectTransformAnchorMin :: KRPCHS.UI.RectTransform -> (Double, Double) -> RPCContext (Bool)
setRectTransformAnchorMin thisArg valueArg = do
    let r = makeRequest "UI" "RectTransform_set_AnchorMin" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Position of the rectangles pivot point relative to the anchors.
 -}
setRectTransformLocalPosition :: KRPCHS.UI.RectTransform -> (Double, Double, Double) -> RPCContext (Bool)
setRectTransformLocalPosition thisArg valueArg = do
    let r = makeRequest "UI" "RectTransform_set_LocalPosition" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Position of the rectangles lower left corner relative to the anchors.
 -}
setRectTransformLowerLeft :: KRPCHS.UI.RectTransform -> (Double, Double) -> RPCContext (Bool)
setRectTransformLowerLeft thisArg valueArg = do
    let r = makeRequest "UI" "RectTransform_set_LowerLeft" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Location of the pivot point around which the rectangle rotates, defined as a fraction of the size of the rectangle itself.
 -}
setRectTransformPivot :: KRPCHS.UI.RectTransform -> (Double, Double) -> RPCContext (Bool)
setRectTransformPivot thisArg valueArg = do
    let r = makeRequest "UI" "RectTransform_set_Pivot" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Position of the rectangles pivot point relative to the anchors.
 -}
setRectTransformPosition :: KRPCHS.UI.RectTransform -> (Double, Double) -> RPCContext (Bool)
setRectTransformPosition thisArg valueArg = do
    let r = makeRequest "UI" "RectTransform_set_Position" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Rotation, as a quaternion, of the object around its pivot point.
 -}
setRectTransformRotation :: KRPCHS.UI.RectTransform -> (Double, Double, Double, Double) -> RPCContext (Bool)
setRectTransformRotation thisArg valueArg = do
    let r = makeRequest "UI" "RectTransform_set_Rotation" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Scale factor applied to the object in the x, y and z dimensions.
 -}
setRectTransformScale :: KRPCHS.UI.RectTransform -> (Double, Double, Double) -> RPCContext (Bool)
setRectTransformScale thisArg valueArg = do
    let r = makeRequest "UI" "RectTransform_set_Scale" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Width and height of the rectangle.
 -}
setRectTransformSize :: KRPCHS.UI.RectTransform -> (Double, Double) -> RPCContext (Bool)
setRectTransformSize thisArg valueArg = do
    let r = makeRequest "UI" "RectTransform_set_Size" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Position of the rectangles upper right corner relative to the anchors.
 -}
setRectTransformUpperRight :: KRPCHS.UI.RectTransform -> (Double, Double) -> RPCContext (Bool)
setRectTransformUpperRight thisArg valueArg = do
    let r = makeRequest "UI" "RectTransform_set_UpperRight" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Remove the UI object.
 -}
textRemove :: KRPCHS.UI.Text -> RPCContext (Bool)
textRemove thisArg = do
    let r = makeRequest "UI" "Text_Remove" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Alignment.
 -}
getTextAlignment :: KRPCHS.UI.Text -> RPCContext (KRPCHS.UI.TextAnchor)
getTextAlignment thisArg = do
    let r = makeRequest "UI" "Text_get_Alignment" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getTextAlignmentStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream (KRPCHS.UI.TextAnchor))
getTextAlignmentStream thisArg = do
    let r = makeRequest "UI" "Text_get_Alignment" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - A list of all available fonts.
 -}
getTextAvailableFonts :: KRPCHS.UI.Text -> RPCContext ([Data.Text.Text])
getTextAvailableFonts thisArg = do
    let r = makeRequest "UI" "Text_get_AvailableFonts" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getTextAvailableFontsStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream ([Data.Text.Text]))
getTextAvailableFontsStream thisArg = do
    let r = makeRequest "UI" "Text_get_AvailableFonts" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Set the color
 -}
getTextColor :: KRPCHS.UI.Text -> RPCContext ((Double, Double, Double))
getTextColor thisArg = do
    let r = makeRequest "UI" "Text_get_Color" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getTextColorStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream ((Double, Double, Double)))
getTextColorStream thisArg = do
    let r = makeRequest "UI" "Text_get_Color" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The text string
 -}
getTextContent :: KRPCHS.UI.Text -> RPCContext (Data.Text.Text)
getTextContent thisArg = do
    let r = makeRequest "UI" "Text_get_Content" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getTextContentStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream (Data.Text.Text))
getTextContentStream thisArg = do
    let r = makeRequest "UI" "Text_get_Content" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Name of the font
 -}
getTextFont :: KRPCHS.UI.Text -> RPCContext (Data.Text.Text)
getTextFont thisArg = do
    let r = makeRequest "UI" "Text_get_Font" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getTextFontStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream (Data.Text.Text))
getTextFontStream thisArg = do
    let r = makeRequest "UI" "Text_get_Font" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Line spacing.
 -}
getTextLineSpacing :: KRPCHS.UI.Text -> RPCContext (Float)
getTextLineSpacing thisArg = do
    let r = makeRequest "UI" "Text_get_LineSpacing" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getTextLineSpacingStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream (Float))
getTextLineSpacingStream thisArg = do
    let r = makeRequest "UI" "Text_get_LineSpacing" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - The rect transform for the text.
 -}
getTextRectTransform :: KRPCHS.UI.Text -> RPCContext (KRPCHS.UI.RectTransform)
getTextRectTransform thisArg = do
    let r = makeRequest "UI" "Text_get_RectTransform" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getTextRectTransformStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream (KRPCHS.UI.RectTransform))
getTextRectTransformStream thisArg = do
    let r = makeRequest "UI" "Text_get_RectTransform" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Font size.
 -}
getTextSize :: KRPCHS.UI.Text -> RPCContext (Data.Int.Int32)
getTextSize thisArg = do
    let r = makeRequest "UI" "Text_get_Size" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getTextSizeStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream (Data.Int.Int32))
getTextSizeStream thisArg = do
    let r = makeRequest "UI" "Text_get_Size" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Font style.
 -}
getTextStyle :: KRPCHS.UI.Text -> RPCContext (KRPCHS.UI.FontStyle)
getTextStyle thisArg = do
    let r = makeRequest "UI" "Text_get_Style" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getTextStyleStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream (KRPCHS.UI.FontStyle))
getTextStyleStream thisArg = do
    let r = makeRequest "UI" "Text_get_Style" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Whether the UI object is visible.
 -}
getTextVisible :: KRPCHS.UI.Text -> RPCContext (Bool)
getTextVisible thisArg = do
    let r = makeRequest "UI" "Text_get_Visible" [makeArgument 0 thisArg]
    res <- sendRequest r
    processResponse extract res 

getTextVisibleStream :: KRPCHS.UI.Text -> RPCContext (KRPCStream (Bool))
getTextVisibleStream thisArg = do
    let r = makeRequest "UI" "Text_get_Visible" [makeArgument 0 thisArg]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


{-
 - Alignment.
 -}
setTextAlignment :: KRPCHS.UI.Text -> KRPCHS.UI.TextAnchor -> RPCContext (Bool)
setTextAlignment thisArg valueArg = do
    let r = makeRequest "UI" "Text_set_Alignment" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Set the color
 -}
setTextColor :: KRPCHS.UI.Text -> (Double, Double, Double) -> RPCContext (Bool)
setTextColor thisArg valueArg = do
    let r = makeRequest "UI" "Text_set_Color" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The text string
 -}
setTextContent :: KRPCHS.UI.Text -> Data.Text.Text -> RPCContext (Bool)
setTextContent thisArg valueArg = do
    let r = makeRequest "UI" "Text_set_Content" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Name of the font
 -}
setTextFont :: KRPCHS.UI.Text -> Data.Text.Text -> RPCContext (Bool)
setTextFont thisArg valueArg = do
    let r = makeRequest "UI" "Text_set_Font" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Line spacing.
 -}
setTextLineSpacing :: KRPCHS.UI.Text -> Float -> RPCContext (Bool)
setTextLineSpacing thisArg valueArg = do
    let r = makeRequest "UI" "Text_set_LineSpacing" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Font size.
 -}
setTextSize :: KRPCHS.UI.Text -> Data.Int.Int32 -> RPCContext (Bool)
setTextSize thisArg valueArg = do
    let r = makeRequest "UI" "Text_set_Size" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Font style.
 -}
setTextStyle :: KRPCHS.UI.Text -> KRPCHS.UI.FontStyle -> RPCContext (Bool)
setTextStyle thisArg valueArg = do
    let r = makeRequest "UI" "Text_set_Style" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - Whether the UI object is visible.
 -}
setTextVisible :: KRPCHS.UI.Text -> Bool -> RPCContext (Bool)
setTextVisible thisArg valueArg = do
    let r = makeRequest "UI" "Text_set_Visible" [makeArgument 0 thisArg, makeArgument 1 valueArg]
    res <- sendRequest r
    processResponse extractNothing res
      


{-
 - The rect transform for the canvas.
 -}
getRectTransform :: RPCContext (KRPCHS.UI.RectTransform)
getRectTransform  = do
    let r = makeRequest "UI" "get_RectTransform" []
    res <- sendRequest r
    processResponse extract res 

getRectTransformStream :: RPCContext (KRPCStream (KRPCHS.UI.RectTransform))
getRectTransformStream  = do
    let r = makeRequest "UI" "get_RectTransform" []
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid 


