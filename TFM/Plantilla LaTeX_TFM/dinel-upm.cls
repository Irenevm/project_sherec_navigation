%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%                          dinel-upm.cls
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% dinel-upm.cls
% Copyright 2015 Hugo Rocha Mendonça
%
% This file belongs to the Department of Electrical Engineering
% (ETSII - UPM), a LaTeX template for writing Thesis and other
% documents.
%
% This work may be distributed and/or modified under the
% conditions of the LaTeX Project Public License, either version 1.3
% of this license or (at your option) any later version.
% The latest version of this license is in
%   http://www.latex-project.org/lppl.txt
% and version 1.3 or later is part of all distributions of LaTeX
% version 2005/12/01 or later.
%
% This work has the LPPL maintenance status `maintained'.
%
% The Current Maintainer of this work is Hugo Rocha Mendonça.
%
% This work consists of the files pig.dtx and pig.ins
% and the derived file pig.sty.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{dinel-upm}[2016/07/08 version 1.3.1 Dinel-ETSII-UPM tesis class]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define nuevos campos (similar a los standards \author,
% \title,\date, etc.):

\def\TitAutor#1{\gdef\@degAuthor{#1}}
\def\Dept#1{\gdef\@dept{#1}}
\def\Director#1{\gdef\@adviser{#1}}
\def\TitDirector#1{\gdef\@degAdviser{#1}}
\def\degree#1{\gdef\@degree{#1}}
\def\authorname#1{\gdef\@authorname{#1}}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define variables para el ambiente <if else>

\newif\iftesis
\newif\iftfm
\newif\iftfg
\newif\ifdoblecara
\newif\ifunacara
\newif\ifcodirector
\newif\ifesim
\newif\ifeum
\newif\ifedob
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Opciones de <documentclass>

% tipos de espacio entre líneas
\DeclareOption{espaciosimple}{\esimtrue\eumfalse\edobfalse}
\DeclareOption{espaciounoymedio}{\esimfalse\eumtrue\edobfalse}
\DeclareOption{espaciodoble}{\esimfalse\eumfalse\edobtrue}
% tipo de trabajo
\DeclareOption{tesis}{\tesistrue\tfmfalse\tfgfalse}
\DeclareOption{tfm}{\tesisfalse\tfmtrue\tfgfalse}
\DeclareOption{tfg}{\tesisfalse\tfmfalse\tfgtrue}
% tipo de impresión
\DeclareOption{unacara}{\doblecarafalse\unacaratrue \PassOptionsToClass{oneside}{book}}
\DeclareOption{doblecara}{\doblecaratrue\unacarafalse \PassOptionsToClass{twoside}{book}}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cualesquiera otras opciones definidas son pasadas a la clase book

\DeclareOption*{\PassOptionsToClass{\CurrentOption}{book}}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Opciones por defecto

\ExecuteOptions{espaciounoymedio,tfg,doblecara}

% Procesa las dadas opciones
\ProcessOptions\relax
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Carga la clase book

\LoadClass[12pt,a4paper]{book}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Lista de paquetes utilizados en esta clase

\RequirePackage{xkeyval}
\RequirePackage[top=2.5cm, bottom=2.5cm, left=3cm, right=2cm]{geometry}
\RequirePackage{setspace}
\RequirePackage[utf8]{inputenc}
\RequirePackage{ifthen}
\RequirePackage{graphicx}
\RequirePackage{titlesec}
\RequirePackage{color}
\RequirePackage{fancyhdr}
\RequirePackage{nomencl}
\RequirePackage[acronym]{glossaries}
\RequirePackage{longtable}
\RequirePackage{epstopdf}
\RequirePackage[format=hang,margin=10pt,font=small,labelfont=bf,labelsep=quad]{caption}
\RequirePackage{subcaption}
\ifunacara  \else \RequirePackage{emptypage} \fi
\RequirePackage{ifpdf}
%\ifpdf
\RequirePackage{hyperref}
\hypersetup{colorlinks=false}
%\else
%\fi
\RequirePackage[backend=bibtex,style=authoryear,hyperref=true,maxbibnames=99,giveninits]{biblatex}
\DefineBibliographyStrings{spanish}{andothers={y~otros}}
\RequirePackage[style=english]{csquotes}

\RequirePackage[spanish,es-lcroman]{babel}
\usepackage{datetime}


\RequirePackage[export]{adjustbox}
\selectlanguage{spanish}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Altura de la cabecera

\setlength{\headheight}{15pt}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Nuevos comandos para creación de líneas

\newcommand{\HRule}{\rule{\linewidth}{0.5mm}}
\newcommand{\bigrule}{\titlerule[1mm]}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Nuevos comandos para codirector

\newcommand{\codirector}[1]{%
\codirectortrue
\newcommand{\codir}{#1}
}

\newcommand{\TitCodirector}[1]{%
\newcommand{\Degcodir}{#1}
}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Nuevo formato del título de los capítulos

\titleformat
{\chapter} % command
[display] % shape
{\bfseries\huge\sffamily} % format
{\normalsize \chaptertitlename \Huge\thechapter\filleft} % label
{-1.5ex} % sep
{
    \rule{\textwidth}{2pt}
    \vspace{0ex}
    \filleft
} % before-code
[
\vspace{-1.5ex}
\rule{\textwidth}{2pt}
] % after-code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Nuevos estilos de cabecera

% texto principal
\fancypagestyle{fancybook}{%
    \fancyhf{}
    \renewcommand*{\sectionmark}[1]{
    \markright{\scriptsize\MakeUppercase{##1}} }
    \renewcommand*{\chaptermark}[1]{
    \markboth{\scriptsize\MakeUppercase{##1}}{} }
    
    
    
    \fancyhead[LE]{\sffamily\scriptsize TRABAJO FINAL DE MÁSTER\\\MakeUppercase{\@title} - \MakeUppercase{\@authorname}}
    \fancyhead[RO]{\sffamily\scriptsize UNIVERSIDAD POLITÉCNICA DE MADRID\\ESCUELA TÉCNICA SUPERIOR DE INGENIEROS INDUSTRIALES}
    \renewcommand{\footrulewidth}{0.5pt}
    \fancyfoot[LE]{\sffamily\textbf{\small\thepage}\quad\leftmark}
    \fancyfoot[RO]{\sffamily\rightmark\quad\textbf{\small\thepage}}
    
}

% redefine el modo plain (utilizado para la primera página de todos
% los capítulos
\newcommand{\cabeceraplain}{%
\fancypagestyle{plain}{%
	\fancyhf{}
	\renewcommand{\headrulewidth}{0pt}
	\renewcommand{\footrulewidth}{0pt}
	\fancyfoot[LE]{\sffamily\textbf{\small\thepage}}
	\fancyfoot[RO]{\sffamily\textbf{\small\thepage}}
}}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Nuevo comando para hoja de título

\newcommand{\titulo}{%
% inicia contador en números romanos
\pagenumbering{roman}

\singlespacing

{\sffamily

\begin{center}

% Cabecera de la Universidad
\onehalfspacing
\begin{figure}[!htb]
   \begin{minipage}{0.48\textwidth}
     \includegraphics[height=2.8cm,left]{figs/portada/UPMlogo}
   \end{minipage}\hfill
   \begin{minipage}{0.48\textwidth}
     \includegraphics[height=2.8cm,right]{figs/portada/logo-etsii}
   \end{minipage}
\end{figure}
\vspace{1cm}
\textsc{\large\textbf{Universidad Politécnica de Madrid}}\\[0.3cm]
\textsc{\large\textbf{Escuela Técnica Superior de Ingenieros Industriales}}\\[0.3cm]
\textsc{\Large\textbf{\@degree}} \\[0.3cm]
\includegraphics[height=3.4cm,center]{figs/portada/Car_4_3.jpg}

\vspace{0.3cm}\textsc{\large \textbf{
\iftesis Tesis Doctoral \else
\iftfm Trabajo Fin de Máster \else
\iftfg Trabajo Fin de Grado \else
Tesis Doctoral
\fi
\fi
\fi} } \\[0.3cm]

% Título
{ \Large \bfseries \@title}\\[3cm]


\ifcodirector
% Autor e director
{\large
\begin{center}
\begin{tabular}{rl}
Autor: & \large{\textbf{\@author}} \\
       & \textit{\@degAuthor} \\
Director 1: & \large{\textbf{\@adviser}} \\
          & \textit{\@degAdviser} \\
Director 2: & \large{\textbf{\codir}} \\
          & \textit{\Degcodir}
\end{tabular}
\end{center}}
\else
% Autor e director
{\large
\begin{center}
\begin{tabular}{rl}
Autor: & \Large{\textbf{\@author}} \\
       & \textit{\@degAuthor} \\
Director: & \Large{\textbf{\@adviser}} \\
          & \textit{\@degAdviser}
\end{tabular}
\end{center}}
\fi


% Fin de la página
\vfill
\singlespacing



{\large \bfseries \monthname, \the\year}

\end{center}
\thispagestyle{empty}
\ifdoblecara \clearpage\mbox{}\thispagestyle{empty}\clearpage \else \fi
}
}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Nuevo comando para hoja de tribunal

\newcommand{\tribunal}[5]{%
% Cabecera de la Universidad
\doublespacing
{\center
\textsc{\large \textbf{UNIVERSIDAD POLITÉCNICA DE MADRID}}\\
\textsc{\large\textbf{Escuela Técnica Superior de Ingenieros Industriales}}\\
\textsc{\large\textbf{Area de conocimiento: Ingeniería Eléctrica}} \\[2cm]}

% Título
\onehalfspacing
\noindent
\ifcodirector
{\large\textbf{TESIS DOCTORAL:} \@title.\\[1.5cm]}

% Autor e director
\begin{center}
{\large
\begin{tabular}{rl}
\textbf{Autor:} & \@author \\
\textbf{Director:} & \@adviser\\
\textbf{Codirector:} & \codir
\end{tabular}}\\[2cm]
\end{center}
\else
{\large\textbf{TESIS DOCTORAL:} \@title.\\[2cm]}

% Autor e director
\begin{center}
{\large
\begin{tabular}{rl}
\textbf{Autor:} & \@author \\
\textbf{Director:} & \@adviser
\end{tabular}}\\[2cm]
\end{center}
\fi

% Personal del Tribunal
\noindent
{\large Tribunal nombrado por el Mgnfco. y Excmo. Sr. Rector de la Universidad Politécnica de Madrid, el día \makebox[1cm]{\hrulefill} de \makebox[3cm]{\hrulefill} de \the\year.}\\[1cm]
{
\begin{tabular}{lll}
\textbf{TRIBUNAL:} &  &  \\[0.5cm]
\textbf{PRESIDENTE:} & D. {#1} & \makebox[5cm]{\hrulefill} \\[0.5cm]
\textbf{VOCALES:} & D. {#3} & \makebox[5cm]{\hrulefill} \\[0.5cm]
 & D. {#4} & \makebox[5cm]{\hrulefill} \\[0.5cm]
  & D. {#5} & \makebox[5cm]{\hrulefill} \\[0.5cm]
  \textbf{SECRETARIO:} & D. {#2} & \makebox[5cm]{\hrulefill} \\[0.5cm]
%  \textbf{SUPLENTES:} & D. \makebox[6cm]{\hrulefill} & \makebox[5cm]{\hrulefill} \\ [0.5cm]
%  & D. \makebox[6cm]{\hrulefill} & \makebox[5cm]{\hrulefill}
  \end{tabular}}\\[0.5cm]

% Realización de la lectura y calificación
  \noindent
  {\large Realizado el acto de lectura y defensa de la Tesis Doctoral, el día \makebox[1cm]{\hrulefill} de \makebox[3cm]{\hrulefill} de \the\year.}\onehalfspacing\\
   \noindent
  \begin{center}
{\large \textbf{Calificación:} \makebox[10cm]{\hrulefill} }
\end{center}
\thispagestyle{empty}
\newpage
\ifdoblecara \newpage\thispagestyle{empty} \else \fi
}%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Nuevo comando para el Indice general

\newcommand{\indices}{%

\cabeceraplain

\ifesim \singlespacing \else
\ifeum \onehalfspacing \else
\ifedob \doublespacing \else
\onehalfspacing
\fi
\fi
\fi

\renewcommand{\contentsname}{\'{I}ndice} % nuevo nombre del índice
\tableofcontents % carga el índice

\pagestyle{fancy} % cabecera ToC
\fancyhf{}
\fancyhead[LE]{\sffamily\textbf{\small\thepage}\quad\MakeUppercase{\scriptsize{\contentsname}}}
\fancyhead[RO]{\sffamily\MakeUppercase{\scriptsize{\contentsname}}\quad\textbf{\small\thepage}}
    \renewcommand{\footrulewidth}{0.5pt}
    \fancyfoot[LE]{\sffamily\scriptsize \MakeUppercase{\@title}\\\MakeUppercase{\@authorname}}
    \fancyfoot[RO]{\sffamily\scriptsize UNIVERSIDAD POLITÉCNICA DE MADRID\\ESCUELA TÉCNICA SUPERIOR DE INGENIEROS INDUSTRIALES}


\renewcommand{\listfigurename}{\'{I}ndice de Figuras}
\listoffigures
\addcontentsline{toc}{chapter}{\listfigurename}
\pagestyle{fancy} % cabecera ToF
\fancyhf{}
\fancyhead[LE]{\sffamily\textbf{\small\thepage}\quad\MakeUppercase{\scriptsize{\listfigurename}}}
\fancyhead[RO]{\sffamily\MakeUppercase{\scriptsize{\listfigurename}}\quad\textbf{\small\thepage}}
    \renewcommand{\footrulewidth}{0.5pt}
    \fancyfoot[LE]{\sffamily\scriptsize \MakeUppercase{\@title}\\\MakeUppercase{\@authorname}}
    \fancyfoot[RO]{\sffamily\scriptsize UNIVERSIDAD POLITÉCNICA DE MADRID\\ESCUELA TÉCNICA SUPERIOR DE INGENIEROS INDUSTRIALES}

\renewcommand{\listtablename}{\'{I}ndice de Tablas}
\renewcommand{\tablename}{Tabla}
\listoftables
\addcontentsline{toc}{chapter}{\listtablename}
\pagestyle{fancy} % cabecera ToT
\fancyhf{}
\fancyhead[LE]{\sffamily\textbf{\small\thepage}\quad\MakeUppercase{\scriptsize{\listtablename}}}
\fancyhead[RO]{\sffamily\MakeUppercase{\scriptsize{\listtablename}}\quad\textbf{\small\thepage}}
    \renewcommand{\footrulewidth}{0.5pt}
    \fancyfoot[LE]{\sffamily\scriptsize \MakeUppercase{\@title}\\\MakeUppercase{\@authorname}}
    \fancyfoot[RO]{\sffamily\scriptsize UNIVERSIDAD POLITÉCNICA DE MADRID\\ESCUELA TÉCNICA SUPERIOR DE INGENIEROS INDUSTRIALES}

}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Nuevo comando para la dedicatoria

\newcommand{\dedicatoria}[1]{%
%\chapter*{}
\newpage
\@input{#1}
\thispagestyle{empty}
}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Nuevo comando para los agradecimientos

\newcommand{\agradecimientos}[1]{%
% configura la primera página de cada capítulo
\cabeceraplain

\chapter*{Agradecimientos}

  \pagestyle{fancy}
  \fancyhf{}
  \fancyhead[LE]{\sffamily\textbf{\small\thepage}\quad\MakeUppercase{\scriptsize{Agradecimientos}}}
  \fancyhead[RO]{\sffamily\MakeUppercase{\scriptsize{Agradecimientos}}\quad\textbf{\small\thepage}}
    \renewcommand{\footrulewidth}{0.5pt}
    \fancyfoot[LE]{\sffamily\scriptsize \MakeUppercase{\@title}\\\MakeUppercase{\@authorname}}
    \fancyfoot[RO]{\sffamily\scriptsize UNIVERSIDAD POLITÉCNICA DE MADRID\\ESCUELA TÉCNICA SUPERIOR DE INGENIEROS INDUSTRIALES}

\ifesim \singlespacing \else
\ifeum \onehalfspacing \else
\ifedob \doublespacing \else
\onehalfspacing
\fi
\fi
\fi

  %\noindent
  \@input{#1}
}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Nuevo comando para el resumen

\newcommand{\resumen}[1]{%

% configura la primera página de cada capítulo
\cabeceraplain

\chapter*{Resumen}

\ifesim \singlespacing \else
\ifeum \onehalfspacing \else
\ifedob \doublespacing \else
\onehalfspacing
\fi
\fi
\fi

  \pagestyle{fancy}
  \fancyhf{}
  \fancyhead[LE]{\sffamily\textbf{\small\thepage}\quad\MakeUppercase{\scriptsize{resumen}}}
  \fancyhead[RO]{\sffamily\MakeUppercase{\scriptsize{resumen}}\quad\textbf{\small\thepage}}
    \renewcommand{\footrulewidth}{0.5pt}
    \fancyfoot[LE]{\sffamily\scriptsize \MakeUppercase{\@title}\\\MakeUppercase{\@authorname}}
    \fancyfoot[RO]{\sffamily\scriptsize UNIVERSIDAD POLITÉCNICA DE MADRID\\ESCUELA TÉCNICA SUPERIOR DE INGENIEROS INDUSTRIALES}

  %\noindent
  \@input{#1}
}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Nuevo comando para el abstract

\newcommand{\abstract}[1]{%

% configura la primera página de cada capítulo
  \cabeceraplain

\chapter*{Abstract}

  \ifesim \singlespacing \else
\ifeum \onehalfspacing \else
\ifedob \doublespacing \else
\onehalfspacing
\fi
\fi
\fi

  \pagestyle{fancy}
  \fancyhf{}
  \fancyhead[LE]{\sffamily\textbf{\small\thepage}\quad\MakeUppercase{\scriptsize{abstract}}}
  \fancyhead[RO]{\sffamily\MakeUppercase{\scriptsize{astract}}\quad\textbf{\small\thepage}}
    \renewcommand{\footrulewidth}{0.5pt}
    \fancyfoot[LE]{\sffamily\scriptsize \MakeUppercase{\@title}\\\MakeUppercase{\@authorname}}
    \fancyfoot[RO]{\sffamily\scriptsize UNIVERSIDAD POLITÉCNICA DE MADRID\\ESCUELA TÉCNICA SUPERIOR DE INGENIEROS INDUSTRIALES}

    %\noindent
  \@input{#1}
}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Nuevo comando para el resumen ejecutivo

\newcommand{\resumenejecutivo}[1]{%

% configura la primera página de cada capítulo
\cabeceraplain

\chapter*{Resumen Ejecutivo}

\ifesim \singlespacing \else
\ifeum \onehalfspacing \else
\ifedob \doublespacing \else
\onehalfspacing
\fi
\fi
\fi

\pagestyle{fancy}
\fancyhf{}
\fancyhead[LE]{\sffamily\textbf{\small\thepage}\quad\MakeUppercase{\scriptsize{resumen ejecutivo}}}
\fancyhead[RO]{\sffamily\MakeUppercase{\scriptsize{resumen ejecutivo}}\quad\textbf{\small\thepage}}
    \renewcommand{\footrulewidth}{0.5pt}
    \fancyfoot[LE]{\sffamily\scriptsize \MakeUppercase{\@title}\\\MakeUppercase{\@authorname}}
    \fancyfoot[RO]{\sffamily\scriptsize UNIVERSIDAD POLITÉCNICA DE MADRID\\ESCUELA TÉCNICA SUPERIOR DE INGENIEROS INDUSTRIALES}

%\noindent
\renewcommand{\thefigure}{R\arabic{figure}}
\setcounter{figure}{0}

\@input{#1}
}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Nuevo comando para Símbolos

\newcommand{\simbolos}{%
\cabeceraplain
\renewcommand{\nomname}{Lista de Símbolos} % nuevo nombre del índice
\printnomenclature
\addcontentsline{toc}{chapter}{\nomname}
\pagestyle{fancy} % cabecera ToS
\fancyhf{}
\fancyhead[LE]{\sffamily\textbf{\small\thepage}\quad\MakeUppercase{\scriptsize{\nomname}}}
\fancyhead[RO]{\sffamily\MakeUppercase{\scriptsize{\nomname}}\quad\textbf{\small\thepage}}
    \renewcommand{\footrulewidth}{0.5pt}
    \fancyfoot[LE]{\sffamily\scriptsize \MakeUppercase{\@title}\\\MakeUppercase{\@authorname}}
    \fancyfoot[RO]{\sffamily\scriptsize UNIVERSIDAD POLITÉCNICA DE MADRID\\ESCUELA TÉCNICA SUPERIOR DE INGENIEROS INDUSTRIALES}
}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Nuevo comando para Acrónimos

\newcommand{\acronimos}{%
\cabeceraplain
\renewcommand{\acronymname}{Lista de Acrónimos}
\printglossaries
\addcontentsline{toc}{chapter}{\acronymname}
\pagestyle{fancy} % cabecera ToA
\fancyhf{}
\fancyhead[LE]{\sffamily\textbf{\small\thepage}\quad\MakeUppercase{\scriptsize{\acronymname}}}
\fancyhead[RO]{\sffamily\MakeUppercase{\scriptsize{\acronymname}}\quad\textbf{\small\thepage}}
    \renewcommand{\footrulewidth}{0.5pt}
    \fancyfoot[LE]{\sffamily\scriptsize \MakeUppercase{\@title}\\\MakeUppercase{\@authorname}}
    \fancyfoot[RO]{\sffamily\scriptsize UNIVERSIDAD POLITÉCNICA DE MADRID\\ESCUELA TÉCNICA SUPERIOR DE INGENIEROS INDUSTRIALES}
}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Nuevo comando para el texto principal

\newcommand{\texto}{%

\ifesim \singlespacing \else
\ifeum \onehalfspacing \else
\ifedob \doublespacing \else
\onehalfspacing
\fi
\fi
\fi

\mainmatter

% reconfigura los subtítulos de las figuras del texto principal
\renewcommand{\thefigure}{\thechapter.\arabic{figure}}
  \setcounter{figure}{0}

% configura la primera página de cada capítulo
\cabeceraplain

% define el estilo de las cabeceras del texto principal
\pagestyle{fancybook}

}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
