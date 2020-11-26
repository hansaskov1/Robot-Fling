create database throwDB;

use throwDB;

create table Throw (
throwID int not null auto_increment,
object char(255) not null,
angle double not null,
speed double not null,
success bool not null,
primary key (throwID)
);

create table Jointpose (
jointposeID int not null,
base double not null,
shoulder double not null,
elbow double not null,
wrist1 double not null,
wrist2 double not null,
wrist3 double not null,
pathNo int not null,
throwID int not null,
foreign key (throwID) references Throw(throwID)
);

create table Jointvelocity (
jointvelocityID int not null,
base double not null,
shoulder double not null,
elbow double not null,
wrist1 double not null,
wrist2 double not null,
wrist3 double not null,
pathNo int not null,
throwID int not null,
foreign key (throwID) references Throw(throwID)
);

create table Toolpose (
toolposeID int not null,
x double not null,
y double not null,
z double not null,
RX double not null,
RY double not null,
RZ double not null,
pathNo int not null,
throwID int not null,
foreign key (throwID) references Throw(throwID)
);

create table Toolvelocity (
toolvelocityID int not null,
x double not null,
y double not null,
z double not null,
RX double not null,
RY double not null,
RZ double not null,
pathNo int not null,
throwID int not null,
foreign key (throwID) references Throw(throwID)
);