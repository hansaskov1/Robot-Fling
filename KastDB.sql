create database kastDB;

use kastDB;

create table Kast (
kastID int not null,
objekt char(255) not null,
vinkel double not null,
hastighed double not null,
success bool not null,
primary key (kastID)
);

create table Jointpose (
jointposeID int not null,
base double not null,
shoulder double not null,
elbow double not null,
wrist1 double not null,
wrist2 double not null,
wrist3 double not null,
stiNr int not null,
kastID int not null,
foreign key (kastID) references Kast(kastID)
);

create table Jointvelocity (
jointvelocityID int not null,
base double not null,
shoulder double not null,
elbow double not null,
wrist1 double not null,
wrist2 double not null,
wrist3 double not null,
stiNr int not null,
kastID int not null,
foreign key (kastID) references Kast(kastID)
);

create table Toolpose (
toolposeID int not null,
x double not null,
y double not null,
z double not null,
RX double not null,
RY double not null,
RZ double not null,
stiNr int not null,
kastID int not null,
foreign key (kastID) references Kast(kastID)
);

create table Toolvelocity (
toolvelocityID int not null,
x double not null,
y double not null,
z double not null,
RX double not null,
RY double not null,
RZ double not null,
stiNr int not null,
kastID int not null,
foreign key (kastID) references Kast(kastID)
);