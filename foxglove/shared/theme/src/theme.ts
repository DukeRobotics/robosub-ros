import { createTheme } from "@mui/material/styles";

const theme = createTheme({
  components: {
    MuiTableRow: {
      styleOverrides: {
        root: {
          "&:last-child td, &:last-child th": {
            border: 0,
          },
        },
      },
    },
  },
});

export default theme;
