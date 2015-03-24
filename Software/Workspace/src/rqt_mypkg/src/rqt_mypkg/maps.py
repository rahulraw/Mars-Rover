from qgmap import *

class gmapsWidget(QtGui.QWidget):
    def __init__(self, parent = None):
        super(gmapsWidget, self).__init__(parent)
        # w = QtGui.QDialog()
        mapsBoxArea = QtGui.QVBoxLayout()
        l = QtGui.QFormLayout()	

        self.addressEdit = QtGui.QLineEdit()
        self.coordsEdit = QtGui.QLineEdit()
        self.destAddressEdit = QtGui.QLineEdit()
        self.destCoordsEdit = QtGui.QLineEdit()

        self.addressEdit.editingFinished.connect(self.goAddress)
        self.coordsEdit.editingFinished.connect(self.goCoords)
        self.destAddressEdit.editingFinished.connect(self.goCoords)
        self.destCoordsEdit.editingFinished.connect(self.goCoords)

        l.addRow('Start Address:', self.addressEdit)
        l.addRow('Start Coords:', self.coordsEdit)
        l.addRow('Dest Address:', self.destAddressEdit)
        l.addRow('Dest Coords:', self.destCoordsEdit)

        self.pb_route = QtGui.QPushButton("Route")

        h = QtGui.QHBoxLayout()
        h.addLayout(l)
        h.addWidget(self.pb_route)

        mapsBoxArea.addLayout(h)

        self.gmap = QGoogleMap(self)
        self.gmap.markerMoved.connect(self.onMarkerMoved)

        mapsBoxArea.addWidget(self.gmap)

        self.gmap.setSizePolicy(
            QtGui.QSizePolicy.MinimumExpanding,
            QtGui.QSizePolicy.MinimumExpanding)

        self.gmap.waitUntilReady()

        self.gmap.centerAt(41.35,2.05)
        self.gmap.setZoom(13)
        coords = self.gmap.centerAtAddress("Pau Casals 3, Santa Coloma de Cervello")
        # Many icons at: https://sites.google.com/site/gmapsdevelopment/
        self.gmap.addMarker("MyDragableMark", *coords, **dict(
            icon="http://google.com/mapfiles/ms/micons/blue-dot.png",
            draggable=True,
            title = "Move me!"
            ))

        self.setLayout(mapsBoxArea)

    def goCoords(self) :
        def resetError() :
            self.coordsEdit.setStyleSheet('')
        try : latitude, longitude = self.coordsEdit.text().split(",")
        except ValueError :
            self.coordsEdit.setStyleSheet("color: red;")
            QtCore.QTimer.singleShot(500, resetError)
        else :
            self.gmap.centerAt(latitude, longitude)
            self.gmap.moveMarker("MyDragableMark", latitude, longitude)

    def goAddress(self) :
        def resetError() :
            self.addressEdit.setStyleSheet('')
        coords = self.gmap.centerAtAddress(self.addressEdit.text())
        if coords is None :
            self.addressEdit.setStyleSheet("color: red;")
            QtCore.QTimer.singleShot(500, resetError)
            return
        self.gmap.moveMarker("MyDragableMark", *coords)
        self.coordsEdit.setText("{}, {}".format(*coords))

    def onMarkerMoved(self, key, latitude, longitude) :
        print("Moved!!", key, latitude, longitude)
        self.coordsEdit.setText("{}, {}".format(latitude, longitude))


