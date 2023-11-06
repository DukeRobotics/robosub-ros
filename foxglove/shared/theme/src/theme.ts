import { createTheme } from "@mui/material/styles";

const theme = createTheme({
  components: {
    MuiTableCell: {
      styleOverrides: {
        root: {
          borderBottom: 0,
        },
      },
    },
  },
});

export default theme;
