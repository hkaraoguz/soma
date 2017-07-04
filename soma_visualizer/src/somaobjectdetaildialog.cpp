#include "somaobjectdetaildialog.h"
#include "ui_somaobjectdetaildialog.h"

SomaObjectDetailDialog::SomaObjectDetailDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SomaObjectDetailDialog)
{
    ui->setupUi(this);

    this->setWindowTitle("SOMA Object Detail");



}
SomaObjectDetailDialog::SomaObjectDetailDialog(QWidget *parent, soma_msgs::SOMAObject somaobject) :
    QDialog(parent),
    ui(new Ui::SomaObjectDetailDialog)
{
    ui->setupUi(this);

    this->setWindowTitle("SOMA Object Detail");

    this->somaobject =somaobject;

    ui->labelObjectID->setText(QString::fromStdString(this->somaobject.id));

    ui->labelObjectType->setText(QString::fromStdString(this->somaobject.type));

    long timestamp = Util::convertSecTimestamptoMSec(this->somaobject.logtimestamp);

    QDateTime dt = Util::calculateUTCDateTimeFromTimestamp(timestamp);

    ui->labelObjectLogtime->setText(dt.toString(datetimeformat));

    ui->textEditObjectMedatada->setReadOnly(true);

    ui->textEditObjectMedatada->setText(QString::fromStdString(this->somaobject.metadata));

    this->imageIndex = 0;

   // ROS_INFO("Number of images %d",this->somaobject.images.size());


    this->loadImage();


}
void SomaObjectDetailDialog::loadImage()
{
    // If we don't have any images stored, just return
    if(this->somaobject.images.size() <= 0) return;

    sensor_msgs::Image animage = somaobject.images[this->imageIndex];

    //There is something weird with this image, just return
    if(animage.height <= 0 || animage.width <= 0) return ;

    QString imagecountstr = QString::number(this->imageIndex+1)+" / "+QString::number(this->somaobject.images.size());

    ui->labelImageCount->setText(imagecountstr);

    if(animage.encoding == "bgr8")
    {

        cv_bridge::CvImagePtr tempcvimage = cv_bridge::toCvCopy(animage,"bgr8");

         cv_bridge::CvImagePtr acvimage  = cv_bridge::cvtColor(tempcvimage,"rgb8");


        QImage image((unsigned char*)acvimage->image.data,animage.width,animage.height,acvimage->image.step,QImage::Format_RGB888);


       // QPixmap map = QPixmap::fromImage(image);


        ui->labelObjectImage->setScaledContents(true);



        ui->labelObjectImage->setPixmap(QPixmap::fromImage(image));
    }


}

SomaObjectDetailDialog::~SomaObjectDetailDialog()
{
    delete ui;
}

void SomaObjectDetailDialog::on_buttonImageLeft_clicked()
{
    if(this->imageIndex > 0 && this->somaobject.images.size()-1 >= this->imageIndex)
        this->imageIndex--;

    this->loadImage();

}

void SomaObjectDetailDialog::on_buttonImageRight_clicked()
{
    if(this->somaobject.images.size()-1 != this->imageIndex)
        this->imageIndex++;


    this->loadImage();

}
