import QtQuick 2.12
import QtQuick.Window 2.12
import QtQuick.Controls 2.12

Rectangle {
    id: view
    visible: true
    width: 2560
    height: 1600
    anchors.centerIn: parent

    signal viewclose ()

    QtObject{
        id:attr
        property int counter;
        Component.onCompleted: {
            counter=3
            countdown.start()
        }
    }
    AnimatedImage {
        id: animated
        source: "qrc:/images/loading.gif"
        anchors.centerIn: parent
        onCurrentFrameChanged: {
            info.text = ("%1/%2").arg(animated.currentFrame).arg(animated.frameCount)
        }
    }
//    Button {
//        anchors.centerIn: parent;
//        text: "关闭";
//        onClicked: {
//            wndCtrl.close();	// 注册后即可使用注册名进行viewer对象调用
//        }
//    }


    Timer{
        id:countdown
        interval: 1000
        repeat: true
        triggeredOnStart: true//这一设置保证了立即触发，如果没有，你会发现有延迟
        onTriggered: {
            attr.counter-=1;
            if(attr.counter<0)
            {
                countdown.stop()
                wndCtrl.close()
            }
        }
        Component.onCompleted: {
            countdown.start()
        }
    }

}
